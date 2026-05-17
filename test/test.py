# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


class SPI:
    """
    Mode-0 SPI master running on the dut.clk domain.
    Pins (in uio_in / uio_out):
        uio_in[0]  = SCK
        uio_in[1]  = MOSI
        uio_in[2]  = CS# (active low)
        uio_out[3] = MISO
    """

    def __init__(self, dut):
        self.dut = dut
        self._v = 0x04  # CSn=1, SCK=0, MOSI=0

    async def _apply(self):
        self.dut.uio_in.value = self._v
        await ClockCycles(self.dut.clk, 1)

    async def begin(self):
        """Drive CS# low and wait."""
        self._v &= ~0x04
        await self._apply()
        await ClockCycles(self.dut.clk, 2)

    async def end(self):
        """Drive CS# high and wait (latches tx_byte -> tx_sr in your SPI slave)."""
        self._v |= 0x04
        await self._apply()
        await ClockCycles(self.dut.clk, 5)

    async def xfer(self, tx_byte):
        """
        Send one byte, return the byte sampled on MISO.
        SCK idle is low (Mode 0).
        """
        miso = 0
        for i in range(7, -1, -1):
            # --- MOSI setup (2 clk) ---
            if (tx_byte >> i) & 1:
                self._v |= 0x02
            else:
                self._v &= ~0x02
            await self._apply()
            await ClockCycles(self.dut.clk, 2)

            # --- SCK rising edge (2 clk) : sample MISO ---
            self._v |= 0x01
            await self._apply()
            await ClockCycles(self.dut.clk, 2)
            miso = (miso << 1) | ((int(self.dut.uio_out.value) >> 3) & 1)

            # --- SCK falling edge (2 clk) ---
            self._v &= ~0x01
            await self._apply()
            await ClockCycles(self.dut.clk, 2)

        return miso


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # 100 ns system clock (10 MHz).  SPI SCK will be ~1.6 MHz (safe for sync chain).
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    spi = SPI(dut)

    # Initial idle state
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0x04  # CSn high
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    def uo():
        return int(dut.uo_out.value)

    def status():
        v = int(dut.uio_out.value)
        return {
            "busy": (v >> 4) & 1,
            "done": (v >> 5) & 1,
            "ovf": (v >> 6) & 1,
            "dbz": (v >> 7) & 1,
        }

    # =====================================================================
    # 1) MUL   :  2.0 * 0.5 = 1.0   (Q8.8: 0x0200 * 0x0080 >>> 8 = 0x0100)
    # =====================================================================
    dut._log.info("Test MUL")
    await spi.begin()
    await spi.xfer(0x10)
    await spi.xfer(0x02)
    await spi.xfer(0x00)  # A = 0x0200
    await spi.xfer(0x11)
    await spi.xfer(0x00)
    await spi.xfer(0x80)  # B = 0x0080
    await spi.xfer(0x20)  # MUL command
    await spi.end()

    await ClockCycles(dut.clk, 2)
    assert uo() == 0x01, f"MUL MSB wrong: {uo():02x}"

    # =====================================================================
    # 2) MAC   :  8 weights = +0.5 (0x08 Q1.4), 8 inputs = 64 (0x40)
    #              result = 8 * (0.5 * 64) = 256  -> 0x0100 Q8.8
    # =====================================================================
    dut._log.info("Test MAC weights + accumulate")
    for i in range(8):
        await spi.begin()
        await spi.xfer(0x13)
        await spi.xfer(i)
        await spi.xfer(0x08)  # +8 = +0.5
        await spi.end()
        await ClockCycles(dut.clk, 2)

    await spi.begin()
    await spi.xfer(0x14)  # MAC start
    for _ in range(8):
        await spi.xfer(0x15)
        await spi.xfer(0x40)  # input value = 64
    await spi.end()

    # Wait until busy drops (MAC finished)
    for _ in range(100):
        if status()["busy"] == 0:
            break
        await ClockCycles(dut.clk, 1)
    else:
        raise AssertionError("MAC busy never went low")

    assert uo() == 0x01, f"MAC MSB wrong: {uo():02x}"

    # Read LSB via SPI (requires CS pulse because tx_sr latches on CS high)
    await spi.begin()
    await spi.xfer(0x31)  # command: prepare LSB in tx_byte
    await spi.end()
    await spi.begin()
    lsb = await spi.xfer(0x00)  # dummy byte shifts out tx_sr
    await spi.end()
    assert lsb == 0x00, f"MAC LSB wrong: {lsb:02x}"

    # =====================================================================
    # 3) DIV   :  4.0 / 2.0 = 2.0   (0x0400 / 0x0200 -> 0x0200)
    # =====================================================================
    dut._log.info("Test DIV")
    await spi.begin()
    await spi.xfer(0x10)
    await spi.xfer(0x04)
    await spi.xfer(0x00)  # A = 0x0400
    await spi.xfer(0x11)
    await spi.xfer(0x02)
    await spi.xfer(0x00)  # B = 0x0200
    await spi.xfer(0x21)  # DIV start
    await spi.end()

    # Divider run-time ≈ 16-17 clk cycles; poll busy.
    for _ in range(100):
        if status()["busy"] == 0:
            break
        await ClockCycles(dut.clk, 1)
    else:
        raise AssertionError("DIV busy never went low")

    assert uo() == 0x02, f"DIV MSB wrong: {uo():02x}"

    # Verify SPI read-back path for MSB
    await spi.begin()
    await spi.xfer(0x30)  # prepare MSB in tx_byte
    await spi.end()
    await spi.begin()
    msb = await spi.xfer(0x00)
    await spi.end()
    assert msb == 0x02, f"DIV MSB SPI read wrong: {msb:02x}"

    # =====================================================================
    # 4) DBZ   :  divide by zero -> result 0x7FFF, dbz flag set
    # =====================================================================
    dut._log.info("Test divide-by-zero")
    await spi.begin()
    await spi.xfer(0x11)
    await spi.xfer(0x00)
    await spi.xfer(0x00)  # B = 0
    await spi.xfer(0x21)
    await spi.end()

    await ClockCycles(dut.clk, 10)
    assert status()["dbz"] == 1, "DBZ flag not set"
    assert uo() == 0x7F, f"DBZ MSB wrong: {uo():02x}"

    # =====================================================================
    # 5) SPI status read (0x3F) – captures persistent flags before clearing
    # =====================================================================
    dut._log.info("Test SPI status read")
    await spi.begin()
    await spi.xfer(0x3F)  # load tx_byte with {ovf,dbz,busy,done,4b0}
    await spi.end()
    await spi.begin()
    stat = await spi.xfer(0x00)
    await spi.end()

    # dbz should have been captured before 0x3F cleared it
    assert ((stat >> 6) & 1) == 1, f"DBZ not set in status byte: {stat:02x}"
    assert ((stat >> 7) & 1) == 0, f"OVF unexpectedly set in status byte: {stat:02x}"

    dut._log.info("All tests passed!")
