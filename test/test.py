"""Register-interface and USB transaction tests for the USB controller."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

import usb_bus

CLK_NS = 20.833333


async def wait_ns(ns):
    await Timer(ns, unit="ns")


async def spi_byte(dut, value):
    received = 0
    for bit in range(7, -1, -1):
        pins = int(dut.ui_in.value)
        pins = (pins & ~0x03) | (((value >> bit) & 1) << 1)
        dut.ui_in.value = pins
        await wait_ns(125)
        dut.ui_in.value = pins | 1
        await wait_ns(125)
        received = (received << 1) | ((int(dut.uio_out.value) >> 5) & 1)
        dut.ui_in.value = pins
    return received


async def spi_write(dut, addr, values):
    dut.ui_in.value = (int(dut.ui_in.value) & ~0x07) | 0x04
    await wait_ns(200)
    dut.ui_in.value = int(dut.ui_in.value) & ~0x04
    await wait_ns(200)
    await spi_byte(dut, addr & 0x7f)
    for value in values:
        await spi_byte(dut, value)
    dut.ui_in.value = int(dut.ui_in.value) | 0x04
    await wait_ns(200)


async def spi_read(dut, addr, count):
    dut.ui_in.value = (int(dut.ui_in.value) & ~0x07) | 0x04
    await wait_ns(200)
    dut.ui_in.value = int(dut.ui_in.value) & ~0x04
    await wait_ns(200)
    await spi_byte(dut, (addr & 0x7f) | 0x80)
    result = []
    for _ in range(count):
        result.append(await spi_byte(dut, 0))
    dut.ui_in.value = int(dut.ui_in.value) | 0x04
    await wait_ns(200)
    return result


async def i2c_scl(dut, value):
    pins = int(dut.ui_in.value)
    dut.ui_in.value = (pins | 0x08) if value else (pins & ~0x08)
    await wait_ns(500)


async def i2c_sda(dut, release):
    pins = int(dut.model_in.value)
    dut.model_in.value = (pins | 0x10) if release else (pins & ~0x10)
    await wait_ns(500)


async def i2c_start(dut):
    await i2c_scl(dut, 1)
    await i2c_sda(dut, 1)
    await i2c_sda(dut, 0)
    await i2c_scl(dut, 0)


async def i2c_stop(dut):
    await i2c_scl(dut, 0)
    await i2c_sda(dut, 0)
    await i2c_scl(dut, 1)
    await i2c_sda(dut, 1)


async def i2c_send_byte(dut, value):
    for bit in range(7, -1, -1):
        await i2c_scl(dut, 0)
        await i2c_sda(dut, (value >> bit) & 1)
        await i2c_scl(dut, 1)
    await i2c_scl(dut, 0)
    await i2c_sda(dut, 1)
    await i2c_scl(dut, 1)
    ack = ((int(dut.uio_in.value) >> 4) & 1) == 0
    await i2c_scl(dut, 0)
    return ack


async def i2c_read_byte(dut, ack):
    value = 0
    await i2c_sda(dut, 1)
    for _ in range(8):
        await i2c_scl(dut, 0)
        await wait_ns(200)
        await i2c_scl(dut, 1)
        value = (value << 1) | ((int(dut.uio_in.value) >> 4) & 1)
    await i2c_scl(dut, 0)
    await i2c_sda(dut, not ack)
    await i2c_scl(dut, 1)
    await i2c_scl(dut, 0)
    await i2c_sda(dut, 1)
    return value


@cocotb.test()
async def test_register_interfaces(dut):
    dut._log.info("48 MHz clock, reset and register-interface checks")
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0x4c  # CS_N, SCL, VBUS
    dut.model_in.value = 0x11  # external SDA released; full-speed idle pullup via model
    dut.model_usb_oe.value = 0
    dut.model_usb_dp.value = 0
    dut.model_usb_dm.value = 0
    dut.model_fs_pullup.value = 1
    dut.model_ls_pullup.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 16)

    assert await spi_read(dut, 0, 1) == [0x5b]
    await spi_write(dut, 1, [0x21])
    assert await spi_read(dut, 1, 1) == [0x21]
    await spi_write(dut, 5, [0x55])
    assert await spi_read(dut, 5, 1) == [0x55]

    # DATA is a non-incrementing FIFO port; LEN resets its shared pointer.
    payload = [0x12, 0x34, 0x56, 0x78, 0xa5, 0x5a, 0xc3, 0x3c]
    await spi_write(dut, 0x0a, payload)
    await spi_write(dut, 9, [len(payload)])
    await spi_write(dut, 9, [0])
    assert await spi_read(dut, 0x0a, len(payload)) == payload

    # I2C register pointer and write/read at the 7-bit 0x58 address.
    await i2c_start(dut)
    assert await i2c_send_byte(dut, 0x58 << 1)
    assert await i2c_send_byte(dut, 0x05)
    assert await i2c_send_byte(dut, 0x2a)
    await i2c_stop(dut)
    assert await spi_read(dut, 5, 1) == [0x2a]

    await i2c_start(dut)
    assert await i2c_send_byte(dut, 0x58 << 1)
    assert await i2c_send_byte(dut, 0x00)
    await i2c_start(dut)
    assert await i2c_send_byte(dut, (0x58 << 1) | 1)
    assert await i2c_read_byte(dut, ack=False) == 0x5b
    await i2c_stop(dut)

    # IRQ W1C: LINE interrupt should have fired on attach; clear it.
    irq = (await spi_read(dut, 3, 1))[0]
    await spi_write(dut, 3, [irq])
    assert (await spi_read(dut, 3, 1))[0] == 0

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_host_in_and_timeout(dut):
    """Host IN transaction with a model device, plus the no-response path."""
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0x4c
    dut.model_in.value = 0x11
    dut.model_usb_oe.value = 0
    dut.model_fs_pullup.value = 1
    dut.model_ls_pullup.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 16)

    # host mode, IRQs enabled
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    payload = [0x12, 0x34, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06]

    # issue IN EP0; the model answers DATA1 + payload
    async def responder():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and len(pkt) == 3, f"bad IN token {pkt}"
        assert pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, payload))
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"

    # recv_packet must be armed before CMD: TX starts ~5 clk after the
    # command byte commits, inside this SPI write's trailing CS high time.
    resp = cocotb.start_soon(responder())
    await spi_write(dut, 6, [0x60])
    await ClockCycles(dut.clk, 4)
    for _ in range(3000):
        await ClockCycles(dut.clk, 20)
        status = (await spi_read(dut, 2, 1))[0]
        if not (status & 0x08):
            break
    await resp
    result = (await spi_read(dut, 7, 1))[0]
    assert result & 0x0f == 0xB, f"RESULT {result:#x}"
    assert (result >> 7) & 1 == 1
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == payload

    # No response at all -> TIMEOUT error code 1.
    await spi_write(dut, 6, [0x60])
    timed_out = False
    for _ in range(3000):
        await ClockCycles(dut.clk, 10)
        if not ((await spi_read(dut, 2, 1))[0] & 0x08):
            timed_out = True
            break
    assert timed_out
    result = (await spi_read(dut, 7, 1))[0]
    assert (result >> 4) & 7 == 1, f"TIMEOUT result {result:#x}"

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_device_setup_out(dut):
    """Device mode: model host sends SETUP + DATA0, DUT must ACK."""
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0x4c
    dut.model_in.value = 0x11
    dut.model_usb_oe.value = 0
    dut.model_fs_pullup.value = 1
    dut.model_ls_pullup.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 16)

    await spi_write(dut, 1, [0x02])       # device, pullup (connect)
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])       # own address 0
    await spi_write(dut, 6, [0x10])       # arm RX on EP0

    setup_data = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x08, 0x00]
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0xD, 0, 0))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, setup_data))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"

    for _ in range(400):
        await ClockCycles(dut.clk, 10)
        if not ((await spi_read(dut, 2, 1))[0] & 0x08):
            break
    result = (await spi_read(dut, 7, 1))[0]
    assert result & 0x0f == 0xD, f"RESULT {result:#x}"
    assert (await spi_read(dut, 8, 1))[0] == 0xD0
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == setup_data
    assert int(dut.contention_count.value) == 0


async def _tb_init(dut, host=True, pullup=True):
    cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0x4c
    dut.model_in.value = 0x11
    dut.model_usb_oe.value = 0
    dut.model_fs_pullup.value = 1 if pullup else 0
    dut.model_ls_pullup.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 16)


async def _wait_done(dut, polls=3000):
    for _ in range(polls):
        await ClockCycles(dut.clk, 10)
        if not ((await spi_read(dut, 2, 1))[0] & 0x08):
            return True
    return False


@cocotb.test()
async def test_host_setup_handshakes(dut):
    """Host SETUP: model ACKs, NAKs, STALLs or stays silent."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])
    await spi_write(dut, 0x0A, [0xaa, 0xbb, 0xcc, 0xdd, 0x11, 0x22, 0x33, 0x44])
    await spi_write(dut, 9, [8])

    async def responder(handshake):
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xD), f"bad SETUP token {pkt}"
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x3), f"bad DATA0 {pkt}"
        assert pkt[1:-2] == [0xaa, 0xbb, 0xcc, 0xdd, 0x11, 0x22, 0x33, 0x44]
        if handshake is not None:
            await ClockCycles(dut.clk, 10)
            await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(handshake)])

    # ACK -> RESULT.PID = ACK
    resp = cocotb.start_soon(responder(0x2))
    await spi_write(dut, 6, [0x00])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0x2

    # NAK -> RESULT.PID = NAK
    resp = cocotb.start_soon(responder(0xA))
    await spi_write(dut, 6, [0x00])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0xA

    # STALL -> RESULT.PID = STALL
    resp = cocotb.start_soon(responder(0xE))
    await spi_write(dut, 6, [0x00])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0xE

    # silent -> TIMEOUT
    resp = cocotb.start_soon(responder(None))
    await spi_write(dut, 6, [0x00])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] >> 4) & 7 == 1

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_host_in_error_paths(dut):
    """Host IN: bad CRC -> BAD, >8 bytes -> OVERFLOW, zero length OK."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    async def responder(packet):
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        if packet is not None:
            await usb_bus.send_packet(dut, packet)
            if packet is not None:
                await usb_bus.recv_packet(dut)  # possibly an ACK from DUT

    # corrupt CRC16 -> ERR=BAD (2), no ACK
    resp = cocotb.start_soon(
        responder([0x80, usb_bus.pid_byte(0xB), 1, 2, 3, 0xde, 0xad]))
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    result = (await spi_read(dut, 7, 1))[0]
    assert (result >> 4) & 7 == 2, f"BAD result {result:#x}"

    # 9-byte data packet -> ERR=OVERFLOW (3)
    payload9 = list(range(9))
    resp = cocotb.start_soon(responder(usb_bus.data_bytes(0xB, payload9)))
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    result = (await spi_read(dut, 7, 1))[0]
    assert (result >> 4) & 7 == 3, f"OVERFLOW result {result:#x}"

    # zero-length DATA1 -> ACK, LEN 0
    async def responder0():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, []))
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"
    resp = cocotb.start_soon(responder0())
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    result = (await spi_read(dut, 7, 1))[0]
    assert result & 0x0f == 0xB
    assert (await spi_read(dut, 9, 1))[0] == 0

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_host_sof(dut):
    """Host SOF: two SOFs ~48000 clk apart with valid CRC5, frame++."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x05])  # HOST | SOF_EN
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    pkt1 = await usb_bus.recv_packet(dut)
    assert pkt1 is not None and len(pkt1) == 3 and pkt1[0] == usb_bus.pid_byte(0x5), f"no SOF {pkt1}"
    frame1 = pkt1[1] | ((pkt1[2] & 7) << 8)
    t1 = cocotb.utils.get_sim_time()
    pkt2 = await usb_bus.recv_packet(dut)
    assert pkt2 is not None and pkt2[0] == usb_bus.pid_byte(0x5), f"no second SOF {pkt2}"
    frame2 = pkt2[1] | ((pkt2[2] & 7) << 8)
    t2 = cocotb.utils.get_sim_time()
    assert frame2 == (frame1 + 1) & 0x7ff, f"frames {frame1}->{frame2}"
    gap_clks = (t2 - t1) / 20833.333
    assert 46000 < gap_clks < 51000, f"SOF gap {gap_clks} clk"
    # CRC5 valid: recompute over the 11 token bits
    tok1 = pkt1[1] | ((pkt1[2] & 7) << 8)
    assert (pkt1[2] >> 3) == usb_bus.crc5(tok1)

    irq = (await spi_read(dut, 3, 1))[0]
    assert irq & 0x02, f"no SOF irq {irq:#x}"
    assert (await spi_read(dut, 0x0B, 2)) == [frame2 & 0xff, (frame2 >> 8) & 7]
    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_host_attach_detect(dut):
    """FS device attach: LINE irq, SE0_LONG clears, DP=1."""
    await _tb_init(dut, pullup=False)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await ClockCycles(dut.clk, 400)  # detached SE0
    await spi_write(dut, 3, [0x0f])
    status = (await spi_read(dut, 2, 1))[0]
    assert status & 0x04, f"SE0_LONG expected {status:#x}"
    assert not (status & 0x02)

    dut.model_fs_pullup.value = 1
    await ClockCycles(dut.clk, 400)
    irq = (await spi_read(dut, 3, 1))[0]
    assert irq & 0x04, f"no LINE irq on attach {irq:#x}"
    status = (await spi_read(dut, 2, 1))[0]
    assert status & 0x02, f"DP expected {status:#x}"
    assert not (status & 0x04), f"SE0_LONG stuck {status:#x}"
    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_device_in_and_control(dut):
    """Device: IN unarmed->NAK, armed->DATA1+ACK->DONE, no-ACK resend,
    STALL, wrong-EP OUT NAK, other-addr ignored, SOF->FRAME."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x02])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    # IN with nothing armed -> NAK + NAK irq
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xA), f"expected NAK {pkt}"
    assert (await spi_read(dut, 3, 1))[0] & 0x08
    await spi_write(dut, 3, [0x0f])

    # arm TX DATA1 8 bytes on EP0, then IN -> data; model ACKs -> DONE
    await spi_write(dut, 0x0A, [0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04])
    await spi_write(dut, 9, [8])
    await spi_write(dut, 6, [0x60])  # op=2 IN armed, DATA1
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xB), f"expected DATA1 {pkt}"
    assert pkt[1:-2] == [0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04]
    await ClockCycles(dut.clk, 10)
    await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
    assert await _wait_done(dut)
    assert (await spi_read(dut, 3, 1))[0] & 0x01
    await spi_write(dut, 3, [0x0f])

    # re-arm: next IN sends data again; without ACK it stays armed
    await spi_write(dut, 0x0A, [0x55] * 8)
    await spi_write(dut, 9, [8])
    await spi_write(dut, 6, [0x60])
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xB)
    # no ACK: a following IN gets the data packet again
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xB), f"expected resend {pkt}"
    await ClockCycles(dut.clk, 10)
    await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
    assert await _wait_done(dut)
    await spi_write(dut, 3, [0x0f])

    # STALL armed -> IN gets STALL
    await spi_write(dut, 6, [0x30])  # op=3 STALL ep0
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xE), f"expected STALL {pkt}"
    await spi_write(dut, 3, [0x0f])

    # RX armed on EP0, OUT token to EP1 -> NAK (must wait for data EOP)
    await spi_write(dut, 6, [0x10])  # op=1 RX arm ep0
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 1))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, [1, 2]))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xA), f"expected NAK {pkt}"
    await spi_write(dut, 3, [0x0f])

    # token to another address -> ignored entirely
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 5, 0))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, [1]))
    pkt = await usb_bus.recv_packet(dut, limit_cycles=800)
    assert pkt is None, f"unexpected response to foreign addr {pkt}"

    # SOF token -> FRAME register + SOF irq (frame 0x123)
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x5, (0x123 >> 4) & 0x7F, 0x123 & 0xF))
    await ClockCycles(dut.clk, 20)
    assert (await spi_read(dut, 0x0B, 2)) == [0x23, 0x01]
    assert (await spi_read(dut, 3, 1))[0] & 0x02

    assert int(dut.contention_count.value) == 0
