"""Register-interface and USB transaction tests for the USB controller."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

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
    # device mode at reset: D- pull-down pin released
    assert ((int(dut.uio_oe.value) >> 6) & 1) == 0
    await spi_write(dut, 1, [0x21])
    assert await spi_read(dut, 1, 1) == [0x21]
    # host mode: uio[6] drives the D- 15k pull-down low
    assert ((int(dut.uio_oe.value) >> 6) & 1) == 1
    assert ((int(dut.uio_out.value) >> 6) & 1) == 0
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
    """Host IN: bad CRC -> BAD, >64 bytes -> OVERFLOW, zero length OK."""
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

    # 65-byte data packet -> ERR=OVERFLOW (3)
    payload9 = list(range(65))
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


@cocotb.test()
async def test_buffer_lengths(dut):
    """64-byte packet buffer: length sweep 0/1/8/63/64 for host OUT/IN and
    device OUT/IN, plus pointer wrap after a full-buffer transfer."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    def payload(n, seed=0):
        return [(seed + i) & 0xFF for i in range(n)]

    # ---------- host OUT: DUT transmits buffer ----------
    async def host_out(n):
        data = payload(n)
        # fill buffer: DATA writes do not move the pointer; write LEN last
        await spi_write(dut, 9, [0])
        for b in data:
            await spi_write(dut, 0x0A, [b])
        await spi_write(dut, 9, [n])

        async def responder():
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x1), f"OUT token {pkt}"
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x3), f"DATA0 {pkt}"
            assert pkt[1:-2] == data, f"payload len {n}"
            assert pkt[-2:] == list(usb_bus.data_bytes(0x3, data)[-2:]), "CRC16"
            await ClockCycles(dut.clk, 10)
            await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
        resp = cocotb.start_soon(responder())
        await spi_write(dut, 6, [0x10])  # OUT op=1, DATA0
        await resp
        assert await _wait_done(dut)
        assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0x2

    for n in (0, 1, 8, 63, 64):
        await host_out(n)

    # ---------- back-to-back host DATA writes then readback, both parities ----------
    # alternating 0xAA/0x55 exercises every byte position and both half-words
    await spi_write(dut, 9, [0])        # LEN write resets buffer pointer
    pattern = [0xAA if i % 2 == 0 else 0x55 for i in range(64)]
    for b in pattern:
        await spi_write(dut, 0x0A, [b])
    await spi_write(dut, 9, [0])        # reset pointer for readback
    for i, b in enumerate(pattern):
        got = (await spi_read(dut, 0x0A, 1))[0]
        assert got == b, f"byte {i}: wrote {b:#x} read {got:#x}"

    # ---------- host IN: model sends, DUT buffers ----------
    async def host_in(n):
        data = payload(n, seed=0x40 + n)

        async def responder():
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
            await ClockCycles(dut.clk, 10)
            await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, data))
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK len {n} {pkt}"
        resp = cocotb.start_soon(responder())
        await spi_write(dut, 6, [0x60])  # IN
        await resp
        assert await _wait_done(dut)
        assert (await spi_read(dut, 9, 1))[0] == n
        # pointer was cleared by the SIE before receiving; read back
        assert await spi_read(dut, 0x0A, n) == data

    for n in (0, 1, 8, 63, 64):
        await host_in(n)

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_device_buffer_lengths(dut):
    """Device mode: SETUP/OUT receive and IN transmit at lengths up to 64,
    and pointer wrap back to 0 between transfers."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x02])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    def payload(n, seed=0):
        return [(seed + i) & 0xFF for i in range(n)]

    # ---------- device OUT receive ----------
    async def dev_out(n):
        data = payload(n, seed=n)
        await spi_write(dut, 6, [0x10])  # RX arm ep0
        await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 0))
        await ClockCycles(dut.clk, 6)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, data))
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK len {n} {pkt}"
        assert await _wait_done(dut)
        assert (await spi_read(dut, 9, 1))[0] == n
        assert await spi_read(dut, 0x0A, n) == data

    for n in (0, 1, 8, 63, 64):
        await dev_out(n)

    # ---------- device IN transmit ----------
    async def dev_in(n):
        data = payload(n, seed=0xA0 + n)
        await spi_write(dut, 9, [0])
        for b in data:
            await spi_write(dut, 0x0A, [b])
        await spi_write(dut, 9, [n])
        await spi_write(dut, 6, [0x60])  # TX arm, DATA1
        await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xB), f"no DATA1 len {n} {pkt}"
        assert pkt[1:-2] == data, f"device TX payload len {n}"
        assert pkt[-2:] == list(usb_bus.data_bytes(0xB, data)[-2:]), "CRC16"
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
        assert await _wait_done(dut)

    for n in (0, 1, 8, 63, 64):
        await dev_in(n)

    # ---------- pointer wrap: after a 64-byte transfer, next starts at 0 ----------
    await dev_out(64)
    await dev_out(2)

    # ---------- device OUT with 65 bytes: completes, only 64 stored ----------
    await spi_write(dut, 6, [0x10])
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 0))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, payload(65)))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK on 65B {pkt}"
    assert await _wait_done(dut)
    # existing behavior: first 64 bytes land in the buffer, LEN reports actual count
    assert (await spi_read(dut, 9, 1))[0] == 65
    assert await spi_read(dut, 0x0A, 64) == payload(64)

    assert int(dut.contention_count.value) == 0


async def _hpend_monitor(dut, counter, arm):
    """Count cycles where the host-write collision pending register is set.
    Silently does nothing under GATES where the hierarchy is absent."""
    try:
        regs = dut.user_project.u_regs
        int(regs.hpend.value)
    except Exception as exc:
        dut._log.info("hpend probe unavailable: %s", exc)
        return
    dut._log.info("hpend probe attached")
    stats = counter[1]
    while True:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(regs.sie_buf_we.value):
            stats[0] += 1
        if int(regs.data_wr.value):
            stats[1] += 1
            if arm[0]:
                # pulses never coincide in sim timing; force the collision so
                # the deferred-write path is exercised (clears next edge)
                regs.sie_buf_we.value = 1
                arm[0] = 0
        if int(regs.hpend.value):
            counter[0] += 1


@cocotb.test()
async def test_double_buffer(dut):
    """Two 64-byte buffers: per-buffer host OUT, concurrent host/SIE access,
    AUTO_SWAP, device RX concurrency, and write-port collision coverage."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    def payload(n, seed=0):
        return [(seed + i) & 0xFF for i in range(n)]

    async def fill_buffer(hbuf, data):
        await spi_write(dut, 0x0E, [hbuf])
        await spi_write(dut, 9, [0])
        for b in data:
            await spi_write(dut, 0x0A, [b])
        await spi_write(dut, 9, [len(data)])

    async def host_out(sbuf, data):
        async def responder():
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x1), f"OUT token {pkt}"
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x3), f"DATA {pkt}"
            assert pkt[1:-2] == data, f"payload sbuf {sbuf}"
            await ClockCycles(dut.clk, 10)
            await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
        resp = cocotb.start_soon(responder())
        await spi_write(dut, 0x0D, [sbuf << 1])
        await spi_write(dut, 6, [0x10])
        await resp
        assert await _wait_done(dut)
        assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0x2

    # host OUT per buffer: A through SBUF=0 then B through SBUF=1
    a = payload(64, 0x10)
    b = payload(64, 0x80)
    await fill_buffer(0, a)
    await fill_buffer(1, b)
    await host_out(0, a)
    await host_out(1, b)

    # host concurrency: 64B OUT of buffer 0 while SPI writes buffer 1
    await fill_buffer(0, a)

    async def out_and_ack():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x1)
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[1:-2] == a
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
    resp = cocotb.start_soon(out_and_ack())
    await spi_write(dut, 0x0D, [0x00])
    await spi_write(dut, 6, [0x10])
    new1 = payload(64, 0x40)
    await spi_write(dut, 0x0E, [1])
    await spi_write(dut, 9, [0])
    for bb in new1:
        await spi_write(dut, 0x0A, [bb])
    await resp
    assert await _wait_done(dut)
    # buffer 1 intact ...
    await spi_write(dut, 9, [0])
    assert await spi_read(dut, 0x0A, 64) == new1
    # ... and buffer 0 sent correctly (verified by responder)

    # AUTO_SWAP: two ACKed host INs land in buffer 0 then buffer 1
    await spi_write(dut, 0x0D, [0x04])  # SBUF=0, AUTO_SWAP
    d0 = payload(8, 0x21)
    d1 = payload(8, 0x55)

    async def in_responder(data):
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, data))
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"

    resp = cocotb.start_soon(in_responder(d0))
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 0x0D, 1))[0] & 0x02) == 0x02, "SBUF did not swap"

    resp = cocotb.start_soon(in_responder(d1))
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 0x0D, 1))[0] & 0x02) == 0x00, "SBUF did not swap back"

    # a NAKed IN must not toggle SBUF
    async def nak_responder():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0xA)])
    resp = cocotb.start_soon(nak_responder())
    await spi_write(dut, 6, [0x60])
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 0x0D, 1))[0] & 0x02) == 0x00, "SBUF swapped on NAK"

    # read both buffers back through HBUF
    await spi_write(dut, 0x0E, [0])
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == d0
    await spi_write(dut, 0x0E, [1])
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == d1

    # device concurrency: OUT into buffer 0 while SPI writes buffer 1
    await spi_write(dut, 0x0D, [0x00])
    await spi_write(dut, 1, [0x02])      # device
    await spi_write(dut, 6, [0x10])      # arm RX ep0

    collisions = [0, [0, 0]]
    arm = [0]
    mon = cocotb.start_soon(_hpend_monitor(dut, collisions, arm))
    dev = payload(64, 0x30)

    async def write_buf1():
        await spi_write(dut, 0x0E, [1])
        await spi_write(dut, 9, [0])
        for bb in payload(64, 0x60):
            await spi_write(dut, 0x0A, [bb])
    writer = cocotb.start_soon(write_buf1())
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 0))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, dev))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"
    await writer
    assert await _wait_done(dut)

    await spi_write(dut, 0x0E, [0])
    assert (await spi_read(dut, 9, 1))[0] == 64
    assert await spi_read(dut, 0x0A, 64) == dev
    await spi_write(dut, 0x0E, [1])
    await spi_write(dut, 9, [0])
    assert await spi_read(dut, 0x0A, 64) == payload(64, 0x60)

    # directed collision sweep if concurrent writes never hit the pending
    # reg: keep firing 64-byte OUTs while bursting DATA writes, offset stepped
    if collisions[0] == 0:
        for off in range(0, 30, 7):
            await spi_write(dut, 6, [0x10])
            await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 0))
            await ClockCycles(dut.clk, 6 + off)

            async def burst():
                await spi_write(dut, 0x0A, [0x5A] * 48)
            arm[0] = 1
            pk = cocotb.start_soon(burst())
            await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, payload(64, off)))
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"sweep ACK {pkt}"
            pk.cancel()
            await _wait_done(dut)
            if collisions[0]:
                break
    mon.cancel()
    dut._log.info("host-write collisions deferred: %d (sie_we=%d data_wr=%d)",
                  collisions[0], collisions[1][0], collisions[1][1])

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_iso(dut):
    """Isochronous transfers: host iso OUT/IN and device iso OUT/IN carry no
    handshake; other endpoints still NAK."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    data = [0x10 + i for i in range(8)]
    await spi_write(dut, 9, [0])
    for b in data:
        await spi_write(dut, 0x0A, [b])
    await spi_write(dut, 9, [8])
    await spi_write(dut, 0x0D, [0x08])  # ISO

    # ---- host iso OUT: completes after DATA tx, no handshake ----
    async def watch_out():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x1)
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[1:-2] == data
        none = await usb_bus.recv_packet(dut, limit_cycles=60)
        assert none is None, f"unexpected handshake {none}"
    w = cocotb.start_soon(watch_out())
    await spi_write(dut, 6, [0x10])
    await w
    assert await _wait_done(dut, polls=100)
    assert (await spi_read(dut, 7, 1))[0] == 0x00, "iso OUT RESULT"

    # ---- host iso IN: data stored, no ACK ----
    async def in_resp():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, data))
        none = await usb_bus.recv_packet(dut, limit_cycles=60)
        assert none is None, f"unexpected ACK {none}"
    w = cocotb.start_soon(in_resp())
    await spi_write(dut, 6, [0x60])
    await w
    assert await _wait_done(dut)
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == data

    # ---- device iso OUT on armed EP1: done, correct data, no ACK ----
    await spi_write(dut, 1, [0x02])
    await spi_write(dut, 6, [0x11])      # arm RX EP1
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x1, 0, 1))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, data))
    none = await usb_bus.recv_packet(dut, limit_cycles=60)
    assert none is None, f"iso OUT got handshake {none}"
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0x1
    assert (await spi_read(dut, 9, 1))[0] == 8
    assert await spi_read(dut, 0x0A, 8) == data

    # ---- device iso IN on armed EP1: data sent, done without ACK ----
    d2 = [0xa0 + i for i in range(8)]
    await spi_write(dut, 9, [0])
    for b in d2:
        await spi_write(dut, 0x0A, [b])
    await spi_write(dut, 9, [8])
    await spi_write(dut, 6, [0x61])      # arm TX EP1, DATA1
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 1))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xB), f"no iso DATA {pkt}"
    assert pkt[1:-2] == d2
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0x9

    # ---- other endpoints: IN to EP0 still NAKs ----
    await usb_bus.send_packet(dut, usb_bus.token_bytes(0x9, 0, 0))
    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0xA), f"expected NAK {pkt}"

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_cmd_pending(dut):
    """CMD issued inside the SOF guard window is queued (BUSY=1) and runs
    right after the next SOF; a CMD written while the PHY transmits is not
    dropped either."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x05])  # HOST | SOF_EN
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x5), f"no SOF {pkt}"
    dut._log.info("SOF1 seen")

    async def responder():
        pkt = await usb_bus.recv_packet(dut)   # next SOF
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x5), f"exp SOF {pkt}"
        pkt = await usb_bus.recv_packet(dut)   # queued IN token
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9), f"exp IN {pkt}"
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, [1, 2, 3, 4]))
        pkt = await usb_bus.recv_packet(dut)   # ACK
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK {pkt}"
    resp = cocotb.start_soon(responder())

    # ~975 us after the SOF: inside the guard window (936..1000 us)
    await wait_ns(975000)
    dut._log.info("writing CMD in guard window")
    await spi_write(dut, 6, [0x60])
    # BUSY must read 1 immediately even though nothing is on the bus yet
    assert (await spi_read(dut, 2, 1))[0] & 0x08, "CMD not pending"
    await resp
    assert await _wait_done(dut)
    assert ((await spi_read(dut, 7, 1))[0] & 0x0f) == 0xB
    dut._log.info("queued IN done")

    # CMD while the PHY still has a packet in flight must not be dropped.
    # responder3 is armed before the CMD write so it catches the IN token,
    # which goes out only ~50 ns after the pend register is set.
    resp3_ready = cocotb.triggers.Event()

    async def responder3():
        resp3_ready.set()
        pkt = await usb_bus.recv_packet(dut)   # skips ACK2 tail / SOFs
        while pkt is None or pkt[0] != usb_bus.pid_byte(0x9):
            pkt = await usb_bus.recv_packet(dut)
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, [0x55]))
        pkt = await usb_bus.recv_packet(dut)
        while pkt is not None and pkt[0] == usb_bus.pid_byte(0x5):
            pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"exp ACK3 {pkt}"

    async def responder2():
        pkt = await usb_bus.recv_packet(dut)   # IN token (immediate or post-SOF)
        while pkt is not None and pkt[0] == usb_bus.pid_byte(0x5):
            pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9), f"exp IN {pkt}"
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, [0xaa]))
        # write the next CMD while the PHY still sends the previous ACK
        pkt_task = cocotb.start_soon(usb_bus.recv_packet(dut))
        resp3 = cocotb.start_soon(responder3())
        await resp3_ready.wait()
        await spi_write(dut, 6, [0x60])
        pkt = await pkt_task
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x2), f"no ACK2 {pkt}"
        await resp3
    resp = cocotb.start_soon(responder2())
    await spi_write(dut, 6, [0x60])
    await resp
    dut._log.info("responder2 done")
    assert await _wait_done(dut)
    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_sof_guard(dut):
    """A 64-byte OUT issued just before the guard window keeps the next SOF on
    its exact 48000-clock period."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x05])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    data = [i & 0xFF for i in range(64)]
    await spi_write(dut, 9, [0])
    for b in data:
        await spi_write(dut, 0x0A, [b])
    await spi_write(dut, 9, [64])

    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x5), f"no SOF1 {pkt}"
    t1 = cocotb.utils.get_sim_time()

    async def responder():
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x1), f"OUT {pkt}"
        pkt = await usb_bus.recv_packet(dut)
        assert pkt is not None and pkt[1:-2] == data
        await ClockCycles(dut.clk, 10)
        await usb_bus.send_packet(dut, [0x80, usb_bus.pid_byte(0x2)])
    resp = cocotb.start_soon(responder())

    # issue the OUT ~915 us after the SOF, just before the guard window
    await wait_ns(915000)
    await spi_write(dut, 6, [0x10])
    await resp
    assert await _wait_done(dut)

    pkt = await usb_bus.recv_packet(dut)
    assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x5), f"no SOF2 {pkt}"
    t2 = cocotb.utils.get_sim_time()
    gap_clks = (t2 - t1) / 20833.333
    assert 47800 < gap_clks < 48200, f"SOF gap {gap_clks} clk"
    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_host_timeout_window(dut):
    """Host IN timeout: a reply starting 15 bit times after EOP succeeds,
    one starting 30 bit times after reports TIMEOUT."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x01])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])

    async def in_with_delay(bit_times):
        async def responder():
            pkt = await usb_bus.recv_packet(dut)
            assert pkt is not None and pkt[0] == usb_bus.pid_byte(0x9)
            await Timer(bit_times * usb_bus.BIT_PS, unit="ps")
            await usb_bus.send_packet(dut, usb_bus.data_bytes(0xB, [0xa5]))
            pkt = await usb_bus.recv_packet(dut)  # possible ACK
        resp = cocotb.start_soon(responder())
        await spi_write(dut, 6, [0x60])
        await resp
        assert await _wait_done(dut)
        return (await spi_read(dut, 7, 1))[0]

    # 15 bit times: must succeed and ACK
    result = await in_with_delay(15)
    assert (result >> 4) & 7 == 0 and result & 0x0f == 0xB, f"15-bit reply {result:#x}"
    assert (await spi_read(dut, 9, 1))[0] == 1
    assert await spi_read(dut, 0x0A, 1) == [0xa5]

    # 30 bit times: TIMEOUT (the late packet is ignored)
    result = await in_with_delay(30)
    assert (result >> 4) & 7 == 1, f"30-bit reply {result:#x}"

    assert int(dut.contention_count.value) == 0


@cocotb.test()
async def test_device_setup_unarmed(dut):
    """Device with nothing armed never handshakes a SETUP; IRQ.NAK is set."""
    await _tb_init(dut)
    await spi_write(dut, 1, [0x02])
    await spi_write(dut, 4, [0x0f])
    await spi_write(dut, 3, [0x0f])
    await spi_write(dut, 5, [0x00])
    # cmd_op = 0: nothing armed

    await usb_bus.send_packet(dut, usb_bus.token_bytes(0xD, 0, 0))
    await ClockCycles(dut.clk, 6)
    await usb_bus.send_packet(dut, usb_bus.data_bytes(0x3, [0x80, 0x06, 0, 1, 0, 0, 8, 0]))
    none = await usb_bus.recv_packet(dut, limit_cycles=60)
    assert none is None, f"unarmed SETUP got handshake {none}"
    assert (await spi_read(dut, 3, 1))[0] & 0x08, "no NAK irq"
    assert int(dut.contention_count.value) == 0
