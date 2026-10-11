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
