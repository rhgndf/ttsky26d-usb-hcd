"""Small bit-level USB full-speed bus helpers for cocotb."""
from cocotb.triggers import Timer, FallingEdge

BIT_PS = 83333


def pid_byte(pid):
    return (((~pid) & 0xF) << 4) | (pid & 0xF)


def crc5(token11):
    crc = 0x1F
    for bit in range(11):
        mix = (crc ^ (token11 >> bit)) & 1
        crc >>= 1
        if mix:
            crc ^= 0x14
    return (~crc) & 0x1F


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        for bit in range(8):
            mix = (crc ^ (byte >> bit)) & 1
            crc >>= 1
            if mix:
                crc ^= 0xA001
    return (~crc) & 0xFFFF


def token_bytes(pid, address, endpoint):
    token = (address << 4) | (endpoint & 0xF)
    c5 = crc5(token)
    return [0x80, pid_byte(pid), token & 0xFF,
            ((token >> 8) & 0x07) | (c5 << 3)]


def data_bytes(pid, payload):
    c16 = crc16(payload)
    return [0x80, pid_byte(pid), *payload, c16 & 0xFF, c16 >> 8]


def packet_bits(packet):
    bits = [(0x80 >> bit) & 1 for bit in range(8)]
    ones = 0
    for byte in packet[1:]:
        for bit in range(8):
            value = (byte >> bit) & 1
            bits.append(value)
            if value:
                ones += 1
                if ones == 6:
                    bits.append(0)
                    ones = 0
            else:
                ones = 0
    return bits


async def wait_tx_packet(dut, limit_cycles=4000):
    seen = False
    for _ in range(limit_cycles):
        await Timer(1000.0 / 48.0, unit="ns")
        active = (int(dut.uo_out.value) >> 4) & 1
        if active:
            seen = True
        elif seen:
            return
    raise AssertionError("timed out waiting for a DUT USB transmit packet")


async def send_packet(dut, packet, bit_period=BIT_PS, jitter_ns=0.0):
    """Drive a packet through the external USB line model; release to FS J."""
    bits = packet_bits(packet)
    dp, dm = 1, 0  # full-speed idle J
    dut.model_usb_oe.value = 3

    async def cell(line_dp, line_dm, index):
        dut.model_usb_dp.value = line_dp
        dut.model_usb_dm.value = line_dm
        delta = int(jitter_ns * 1000) if index % 2 == 0 else -int(jitter_ns * 1000)
        await Timer(bit_period + delta, unit="ps")

    for index, bit in enumerate(bits):
        if bit == 0:
            dp, dm = (0, 1) if dp else (1, 0)
        await cell(dp, dm, index)
    for index in range(2):
        await cell(0, 0, len(bits) + index)
    await cell(1, 0, len(bits) + 2)
    dut.model_usb_oe.value = 0
    dut.model_fs_pullup.value = 1
    await Timer(4 * 20834, unit="ps")


async def recv_packet(dut, limit_cycles=6000):
    """Decode one DUT-driven packet. Returns destuffed byte list, or None."""
    step = 5000  # ~16.7 samples per bit cell, incommensurate with clk edges
    started = False
    samples = []
    for _ in range(limit_cycles * 4):
        await Timer(step, unit="ps")
        pins = int(dut.uio_in.value)
        samples.append((pins & 1, (pins >> 1) & 1))
        if (int(dut.uo_out.value) >> 4) & 1:
            started = True
            break
    if not started:
        return None
    for _ in range(16 * 64 + 128):
        await Timer(step, unit="ps")
        pins = int(dut.uio_in.value)
        samples.append((pins & 1, (pins >> 1) & 1))
        if not ((int(dut.uo_out.value) >> 4) & 1) and (int(dut.uio_oe.value) & 3) == 0:
            break

    # Keep samples from the first SYNC K through the EOP SE0.
    start = 0
    while start < len(samples) and samples[start] == (1, 0):
        start += 1
    se0 = start
    while se0 < len(samples) and samples[se0] != (0, 0):
        se0 += 1
    cells = samples[start:se0]
    if len(cells) < 64:
        return None

    # Edge-interval decode: a transition every ~16.7 samples is a 0 bit;
    # each extra ~16.7 samples of hold is a 1 bit.
    bits = [0]
    prev = 0
    for i in range(1, len(cells)):
        if cells[i] != cells[i - 1]:
            gap = i - prev
            bits += [1] * (max(1, round(gap / 16.67)) - 1)
            bits.append(0)
            prev = i
    bits += [1] * max(0, round((len(cells) - prev) / 16.67) - 1)

    out = []
    byte = 0
    count = 0
    ones = 0
    for bit in bits[8:]:  # drop the SYNC byte
        if ones == 6:
            if bit:
                return None
            ones = 0
            continue
        byte |= bit << count
        count += 1
        ones = ones + 1 if bit else 0
        if count == 8:
            out.append(byte)
            byte = 0
            count = 0
    return out
