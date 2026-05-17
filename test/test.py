# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


# -------------------------------------------------------------------------
# Raw-pin SPI helpers (Mode-0, MSB-first, slow-as-molasses)
# -------------------------------------------------------------------------
# Pin mapping to your uio_in / uio_out:
#   uio_in[0]  = SCK      (output from testbench)
#   uio_in[1]  = MOSI     (output from testbench)
#   uio_in[2]  = CSn      (output from testbench, active low)
#   uio_out[3] = MISO     (input to testbench)
# -------------------------------------------------------------------------

async def spi_raw_byte(dut, tx_byte):
    """
    Send one byte and return the MISO bits clocked out by the slave.
    CS must already be low when this is called.
    """
    miso = 0
    for i in range(7, -1, -1):
        # MOSI setup (hold 4 sys-clocks before SCK rises)
        dut.uio_in[1].value = (tx_byte >> i) & 1
        await ClockCycles(dut.clk, 4)

        # SCK rise, hold 4 clocks
        dut.uio_in[0].value = 1
        await ClockCycles(dut.clk, 4)

        # sample MISO while SCK is high
        miso = (miso << 1) | (int(dut.uio_out[3].value) & 1)

        # SCK fall, hold 4 clocks
        dut.uio_in[0].value = 0
        await ClockCycles(dut.clk, 4)
    return miso


async def spi_tx_cmd(dut, cmd_byte, data_byte=None):
    """
    Single transaction: CS low -> send cmd_byte -> optional data_byte -> CS high.
    """
    dut.uio_in.value = 0x04          # CSn=1, SCK=0, MOSI=0
    await ClockCycles(dut.clk, 5)

    dut.uio_in[2].value = 0          # CSn low
    await ClockCycles(dut.clk, 5)

    await spi_raw_byte(dut, cmd_byte)

    if data_byte is not None:
        await spi_raw_byte(dut, data_byte)

    dut.uio_in[2].value = 1          # CSn high
    dut.uio_in[1].value = 0          # MOSI back to idle 0
    await ClockCycles(dut.clk, 10)


async def spi_read_uo(dut):
    """u_out is live combinatorial, but async to our Python."""
    return int(dut.uo_out.value)


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # 100 ns period => 10 MHz system clock
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())

    # idle
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0x04          # CSn=1 SCK=0 MOSI=0

    # Reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 15)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # ---------------------------------------------------------------------
    # Helper aliases for internal visibility (Icarus keeps these)
    # ---------------------------------------------------------------------
    async def peek(name):
        """Return integer value of an internal signal if VPI exposes it."""
        try:
            return int(getattr(dut.user_project, name).value)
        except Exception:
            return None

    # ---------------------------------------------------------------------
    # 1) Load A = 0x0200  (+2.0 in Q8.8)
    # ---------------------------------------------------------------------
    await spi_tx_cmd(dut, 0x10, 0x02)   # command HI
    await spi_tx_cmd(dut, 0xAA, 0x00)   # dummy command so FSM sees HI byte in P_A_HI
    # Oops — above is wrong.  We need to send the HI byte while FSM is in P_A_HI.
    # Let's do it properly: send 0x10 in one frame, then send HI, then LO.
    # ---------------------------------------------------------------------

    # Actually let's restart cleanly:
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # 0x10 : next byte -> tmp_hi, goto P_A_LO
    # 0x02 : tmp_hi = 0x02
    # 0x00 : reg_a  = {0x02,0x00}
    await spi_tx_cmd(dut, 0x10)
    await spi_tx_cmd(dut, 0x02)
    await spi_tx_cmd(dut, 0x00)

    ra = await peek("reg_a")
    dut._log.info(f"After loading A, reg_a = {ra}")
    assert ra == 0x0200, f"reg_a loading failed: {ra}"

    # ---------------------------------------------------------------------
    # 2) Load B = 0x0080  (+0.5 in Q8.8)
    # ---------------------------------------------------------------------
    await spi_tx_cmd(dut, 0x11)
    await spi_tx_cmd(dut, 0x00)
    await spi_tx_cmd(dut, 0x80)

    rb = await peek("reg_b")
    dut._log.info(f"After loading B, reg_b = {rb}")
    assert rb == 0x0080, f"reg_b loading failed: {rb}"

    # ---------------------------------------------------------------------
    # 3) MUL  (A * B) >>> 8  =>  2.0 * 0.5 = 1.0  => 0x0100
    # ---------------------------------------------------------------------
    await spi_tx_cmd(dut, 0x20)

    await ClockCycles(dut.clk, 5)
    res = await peek("result")
    uo  = await spi_read_uo(dut)
    dut._log.info(f"After MUL: result={res}, uo_out={uo}")
    assert res == 0x0100, f"MUL result wrong: {res}"
    assert uo == 0x01,    f"uo_out MSB wrong: {uo}"

    # ---------------------------------------------------------------------
    # 4) Load A = 4.0 (0x0400), B = 2.0 (0x0200), DIV => 2.0 (0x0200)
    # ---------------------------------------------------------------------
    await spi_tx_cmd(dut, 0x10)
    await spi_tx_cmd(dut, 0x04)
    await spi_tx_cmd(dut, 0x00)

    await spi_tx_cmd(dut, 0x11)
    await spi_tx_cmd(dut, 0x02)
    await spi_tx_cmd(dut, 0x00)

    await spi_tx_cmd(dut, 0x21)         # DIV start

    # Poll internal busy via uio_out[4] or internal signal
    for _ in range(50):
        busy = int(dut.uio_out[4].value)
        if not busy:
            break
        await ClockCycles(dut.clk, 1)
    else:
        raise AssertionError("DIV busy never cleared")

    res = await peek("result")
    uo  = await spi_read_uo(dut)
    dut._log.info(f"After DIV: result={res}, uo_out={uo}")
    assert res == 0x0200, f"DIV result wrong: {res}"
    assert uo == 0x02,    f"DIV uo_out wrong: {uo}"

    # ---------------------------------------------------------------------
    # 5) DBZ test: A / 0 => saturation + DBZ flag
    # ---------------------------------------------------------------------
    await spi_tx_cmd(dut, 0x11)
    await spi_tx_cmd(dut, 0x00)
    await spi_tx_cmd(dut, 0x00)         # B = 0

    await spi_tx_cmd(dut, 0x21)

    await ClockCycles(dut.clk, 20)      # combinational DBZ is fast
    dbz = int(dut.uio_out[7].value)
    uo  = await spi_read_uo(dut)
    dut._log.info(f"After DBZ: dbz={dbz}, uo_out={uo}")
    assert dbz == 1,       f"DBZ flag not set: {dbz}"
    assert uo == 0x7F,     f"DBZ saturation MSB wrong: {uo}"

    # ---------------------------------------------------------------------
    # 6) MAC smoke test (8 weights = +0.5, 8 inputs = 64)
    #    8 * (0.5 * 64) = 256  =>  0x0100
    # ---------------------------------------------------------------------
    # load weights first
    for i in range(8):
        await spi_tx_cmd(dut, 0x13, i)      # idx
        await spi_tx_cmd(dut, 0x55, 0x08)   # data weight=+8 (Q1.4 => +0.5)

    # MAC start
    await spi_tx_cmd(dut, 0x14)             # clear accumulator, assert busy

    # pump 8 data bytes
    for _ in range(8):
        await spi_tx_cmd(dut, 0x15, 0x40)   # input = 64

    await ClockCycles(dut.clk, 5)
    res = await peek("result")
    uo  = await spi_read_uo(dut)
    dut._log.info(f"After MAC: result={res}, uo_out={uo}")
    assert res == 0x0100, f"MAC result wrong: {res}"
    assert uo == 0x01,    f"MAC uo_out wrong: {uo}"

    dut._log.info("All tests passed!")
