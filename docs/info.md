## How it works

A USB 1.1 **full-speed (12 Mb/s) host *and* device controller** in a single
Tiny Tapeout tile. A host processor (e.g. a Linux SBC or a microcontroller)
drives it through an 8-bit register map over **SPI** or **I2C**; both slave
interfaces are always active and see the same registers. The register map is
designed for the Linux `regmap` API (8-bit registers, 8-bit values,
auto-incrementing bursts, one non-incrementing FIFO port), so a Linux HCD /
UDC driver is a thin layer on top of it.

The chip does everything that is timing-critical in hardware: 4x oversampled
receive with clock recovery, NRZI, bit (un)stuffing, SYNC/EOP, CRC5/CRC16
generation and checking, token/handshake generation, turnaround timeouts,
1 ms SOF generation, address/endpoint matching and attach/reset detection.
Software does everything else (enumeration, data toggles, NAK retries,
scheduling). The packet buffer is built in tnt's (smunaut's) `rf_top`
register-file macro (as used by femtorv-soc, Apache-2.0), 32 words x 32 bits
= 128 bytes arranged as two independent 64-byte buffers. USB PHY:
ultraembedded `core_usb_fs_phy` (LGPL-2.1,
github.com/ultraembedded/core_usb_fs_phy @ 68a63ff937e58e90f4f534b9c6705c649d545136). Each buffer supports
`wMaxPacketSize` <= 64; larger packets overflow.

The clock must be **48 MHz** (4 samples per USB bit, 0.25 % accuracy).

### USB pins and roles

D+ and D- are driven directly by two bidirectional pins. Two more pins make
the role fully software controlled, so one board works as host or device:

| Pin | Host mode (`CTRL.HOST=1`) | Device mode (`CTRL.HOST=0`) |
|-----|---------------------------|------------------------------|
| `uio[2]` PULLUP | Hi-Z | drives 1 when `CTRL.PULLUP=1` (connect), else Hi-Z (disconnect). Wire 1.5 kΩ to D+. |
| `uio[3]` PULLDOWN_DP | drives 0: enables the D+ 15 kΩ host pull-down | Hi-Z |
|| `uio[6]` PULLDOWN_DM | drives 0: enables the D- 15 kΩ host pull-down | Hi-Z |

Attach / detach and speed are sensed digitally from the line state
(`STATUS.DP`, `STATUS.DM`) and the SE0 timer (`STATUS.SE0_LONG`, set when SE0
has lasted >= 2.67 µs): in host mode, `SE0_LONG=0` means a device is attached,
and the idle state tells its speed (`DP=1` full speed, `DM=1` low speed — low
speed is detected but not supported). In device mode `SE0_LONG=1` means the
host is driving a bus reset. Every change of `SE0_LONG` raises `IRQ.LINE`.

### Register map

Only the low 4 address bits are decoded (0x10-0x7F alias 0x00-0x0F).
Unlisted registers read 0. Reset value of every register is 0 unless noted.

| Addr | Name | Access | Bits |
|------|------|--------|------|
| 0x0 | `ID` | R | constant `0x5B` |
| 0x1 | `CTRL` | RW | [0] `HOST` 1=host, 0=device · [1] `PULLUP` device connect · [2] `SOF_EN` host SOF generation · [3] `FORCE_SE0` drive SE0 (host bus reset) · [4] `FORCE_K` drive K (resume / remote wakeup) · [5] `VBUS_EN` drives `uo[1]` |
| 0x2 | `STATUS` | R | [0] `DM` · [1] `DP` (synchronised line state) · [2] `SE0_LONG` · [3] `BUSY` · [4] `VBUS` (`ui[6]`) |
| 0x3 | `IRQ` | R/W1C | [0] `DONE` · [1] `SOF` · [2] `LINE` · [3] `NAK` |
| 0x4 | `IRQ_EN` | RW | same bits as `IRQ`; `uo[0]` INT = OR of (`IRQ` & `IRQ_EN`), active high |
| 0x5 | `ADDR` | RW | [6:0] host: target device address; device: own address |
| 0x6 | `CMD` | RW | [3:0] `EP` · [5:4] `OP` · [6] `DATA1` · [7] `ANY` — writing starts an operation, see below |
| 0x7 | `RESULT` | R | [3:0] `PID` · [6:4] `ERR` · [7] `RX_DATA1` |
| 0x8 | `TOKEN` | R | device: last token addressed to us: [3:0] EP · [7:4] PID |
| 0x9 | `LEN` | RW | [6:0] packet length 0-64 for `HBUF` (values >64 undefined). Writing resets the host pointer to 0 |
| 0xA | `DATA` | RW | packet buffer port: accesses `buf[ptr]`, then `ptr = (ptr+1) mod 64`. Non-incrementing in bursts |
| 0xB | `FRAME_L` | R | frame number [7:0]: host: last SOF sent; device: last SOF received |
| 0xC | `FRAME_H` | R | [2:0] frame number [10:8] |

`IRQ` bits are set by hardware and cleared by writing 1 (a set event in the
same cycle wins). `DONE`: operation completed, `RESULT` valid. `SOF`: host:
SOF sent; device: SOF received (`FRAME` updated). `LINE`: `SE0_LONG` changed.
`NAK`: device only, a token addressed to us was NAKed or ignored (`TOKEN`
says which; useful to learn what the host is asking for).

PIDs: OUT=0x1, IN=0x9, SOF=0x5, SETUP=0xD, DATA0=0x3, DATA1=0xB, ACK=0x2,
NAK=0xA, STALL=0xE.

`RESULT.ERR`: 0 `OK`, 1 `TIMEOUT` (no response started within ~18 bit times),
2 `BAD` (CRC, bit-stuff, PID-check or framing error), 3 `OVERFLOW` (data
packet longer than 64 bytes), 4 `UNEXPECTED` (valid packet of the wrong type).

#### Host mode (`CTRL.HOST=1`)

Write `CMD` with `OP`: 0 `SETUP`, 1 `OUT`, 2 `IN` (3 is ignored) for
endpoint `EP` of device `ADDR`. `STATUS.BUSY` is set until the transaction
ends, then `IRQ.DONE` is raised. Writes to `CMD` while busy are ignored.

* `SETUP` / `OUT`: sends the token, then DATA0 (`SETUP` always uses DATA0) or
  DATA0/DATA1 per `CMD.DATA1` with `LEN` bytes from the buffer, then waits for
  the handshake. `RESULT.PID` = ACK / NAK / STALL.
* `IN`: sends the token and waits for the reply. A good DATA0/DATA1 packet is
  stored in the buffer (`LEN` = byte count, pointer reset) and ACKed;
  `RESULT.PID` is the received data PID — software checks it against the
  expected toggle (on a mismatch the data is a retransmission and must be
  discarded). NAK / STALL are reported in `RESULT.PID`. Bad or overlong
  packets are not ACKed.
* NAKs are not retried by hardware: re-issue the command.
* With `CTRL.SOF_EN=1` a SOF with an incrementing 11-bit frame number is sent
  every 48000 clocks (1 ms). A command issued in the last ~64 µs of a frame
  is queued and runs right after the SOF, so transactions never collide
  with SOFs.
* Bus reset: set `CTRL.FORCE_SE0` for 10-20 ms (software-timed), then clear
  it and set `SOF_EN` (SOFs are the keep-alive that prevents suspend).

#### Device mode (`CTRL.HOST=0`)

The device answers tokens for address `ADDR` according to the *armed*
operation in `CMD.OP` (`EP` is the endpoint it applies to):

| `OP` | SETUP (any EP) | OUT to `EP` (any EP if `ANY`) | IN to `EP` | other OUT / IN |
|------|----------------|-------------------------------|------------|----------------|
| 0 `IDLE` | no handshake (host retries), `NAK` irq | NAK | NAK | NAK |
| 1 `RX` | accept, ACK | accept, ACK | NAK | NAK |
| 2 `TX` | accept, ACK (TX data discarded) | NAK | send `LEN` bytes as DATA0/1 per `CMD.DATA1` | NAK |
| 3 `STALL` | accept, ACK | STALL | STALL | NAK |

When an armed operation completes (a packet was accepted and ACKed, or the
host ACKed our IN data), hardware sets `CMD.OP=0`, fills `RESULT` (`PID` =
the token PID SETUP/OUT/IN, `RX_DATA1` = toggle of received data) and
`TOKEN`, sets `LEN` for received data and raises `IRQ.DONE`. Corrupted
packets are ignored (no handshake, the host retries). Data toggles are
software's job: if `RX_DATA1` is not the expected toggle the packet is a
retransmission whose ACK was lost — just re-arm. If the host does not ACK
our IN data the operation stays armed and the same data is resent on the
next IN. Isochronous transfers are not supported.

Typical EP0 control flow: arm `RX`; on SETUP, decode it; for an IN data
stage arm `TX` EP0 with 8-byte chunks (DATA1, DATA0, ...); status stage:
arm `RX` (host sends a zero-length OUT) or arm `TX` with `LEN=0` DATA1. Write
`ADDR` only after the SET_ADDRESS status stage completed. Unsupported
request: arm `STALL` EP0. On `IRQ.LINE` with `SE0_LONG=1` (bus reset) set
`ADDR=0` and re-arm `RX`.

### Host interfaces

**SPI** (mode 0, MSB first, SCK <= 4 MHz): pull `CS_N` low, send
`{RW, A[6:0]}` (RW=1 read), then data bytes. The address auto-increments
after each data byte, except at `DATA` (0xA) which stays put so a burst
streams the buffer. Linux: `regmap` with `read_flag_mask = 0x80`.
`CIPO` is only driven while `CS_N` is low.

**I2C** (up to 1 MHz, 7-bit address `0b10110 A1 A0` = 0x58-0x5B selected by
`ui[5:4]`): standard register-pointer protocol — write `[reg] [data...]`, or
write `[reg]` then repeated-START read. Same auto-increment rules as SPI. No
clock stretching.

## How to test

Clock 48 MHz. Connect D+/D- of a USB connector to `uio[0]`/`uio[1]` through
22-33 Ω series resistors, 1.5 kΩ from `uio[2]` to D+, 15 kΩ from D+ to
`uio[3]` and 15 kΩ from D- to `uio[6]`.

Device-mode smoke test (over SPI or I2C): write `CTRL=0x02` (device, connect),
`IRQ_EN=0x07`, `CMD=0x10` (arm RX). Plug into a PC: you will see `IRQ.LINE`
(bus reset), then `IRQ.DONE` with `RESULT.PID=0xD` and an 8-byte SETUP
packet in the buffer (read `LEN` then 8x `DATA`).

Host-mode smoke test: write `CTRL=0x01` (host), plug in a full-speed device,
wait for `IRQ.LINE` with `STATUS.SE0_LONG=0, DP=1`, set `CTRL=0x09` (bus
reset) for 20 ms, then `CTRL=0x05` (SOF on), write the 8-byte
GET_DESCRIPTOR(device, 8) request to the buffer (`LEN=8`, 8x `DATA`), `ADDR=0`,
`CMD=0x00` (SETUP EP0) and expect `RESULT.PID=ACK`; then `CMD=0x60` (IN
DATA1) until `RESULT.PID=DATA1` and read the descriptor from the buffer.

## External hardware

USB connector, 2x 22-33 Ω series resistors, 1x 1.5 kΩ, 2x 15 kΩ. For host
mode a 5 V VBUS switch controlled by `uo[1]` (VBUS_EN). For device mode,
optionally VBUS through a divider to `ui[6]`. A SPI or I2C master (I2C needs
pull-ups on SCL/SDA).
