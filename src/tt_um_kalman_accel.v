/*
 * tt_um_kalman_accel.v — Kalman Filter Math Coprocessor (Project B)
 *
 * Tiny Tapeout top-level module.
 *
 * This chip accelerates the expensive fixed-point arithmetic at the heart of
 * a scalar (1D) Kalman filter running on the RP2040: multiply-accumulate and
 * division.  The RP2040 does all the control flow; the chip handles the math.
 *
 * ── Fixed-Point Format ────────────────────────────────────────────────────
 * All values are Q8.8 signed (16-bit two's complement).
 * Range:  -128.0  to  +127.996  (resolution ~0.004)
 * This is good for sensor values, covariances, and gains in a typical
 * accelerometer / temperature / pressure Kalman filter.
 *
 * ── SPI Protocol ──────────────────────────────────────────────────────────
 * Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit bytes.
 *
 * Every transaction is:  CMD  DATA_HI  DATA_LO  [DATA_HI  DATA_LO ...]
 *
 * Commands (1 byte):
 *   0x10  LOAD_A       — Load 16-bit operand A  (2 data bytes, MSB first)
 *   0x11  LOAD_B       — Load 16-bit operand B
 *   0x12  LOAD_C       — Load 16-bit operand C  (used in compound expressions)
 *   0x20  OP_MUL       — Compute result = A * B  (ready next CS cycle)
 *   0x21  OP_DIV       — Compute result = A / B  (takes 17 clk cycles; poll BUSY)
 *   0x22  OP_MACDIV    — Compute result = (A*B) / (C*B + A)  — Kalman gain K
 *                        (K = P*H / (H*P*H + R), scalar, H=1 → K = P/(P+R))
 *                        Here operands: A=P (variance), B=H (=1.0 typically),
 *                        C=R (measurement noise). Simplifies to A/(A+C).
 *   0x30  READ_RESULT  — Clock out 2 bytes (result MSB, LSB) on MISO
 *   0x31  READ_STATUS  — Clock out 1 byte: [ovf, dbz, busy, done, 4'b0]
 *
 * ── Pin Mapping ───────────────────────────────────────────────────────────
 * ui_in[0]  — (unused, tie low)
 * ui_in[7:1] — (unused)
 *
 * uio[0]   — SPI SCK   (input)
 * uio[1]   — SPI MOSI  (input)
 * uio[2]   — SPI CS#   (input, active low)
 * uio[3]   — SPI MISO  (output)
 * uio[4]   — BUSY      (output, high while divider is running)
 * uio[5]   — DONE      (output, pulses high when result ready)
 * uio[6]   — OVF       (output, overflow flag, cleared on new operation)
 * uio[7]   — DBZ       (output, divide-by-zero flag)
 *
 * uo_out[7:0] — result[15:8] (high byte of last result, for quick polling)
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_kalman_accel (
    input  wire [7:0] ui_in,    // dedicated inputs (unused)
    output wire [7:0] uo_out,   // result high byte for easy polling
    inout  wire [7:0] uio,      // SPI bus + status pins
    input  wire       ena,      // design enable from TT mux
    input  wire       clk,      // system clock from RP2040
    input  wire       rst_n     // active-low reset
);

    // ----------------------------------------------------------------
    // Pin direction declarations (required by Tiny Tapeout)
    // uio[3:0] mixed: [3]=output MISO, [2:0]=inputs
    // uio[7:4] outputs
    // ----------------------------------------------------------------
    wire spi_sck  = uio[0];
    wire spi_mosi = uio[1];
    wire spi_cs_n = uio[2];
    wire spi_miso;

    wire busy_out, done_out, ovf_out, dbz_out;

    assign uio[3] = spi_miso;
    assign uio[4] = busy_out;
    assign uio[5] = done_out;
    assign uio[6] = ovf_out;
    assign uio[7] = dbz_out;

    // ================================================================
    // SPI Slave
    // ================================================================
    wire [7:0] spi_rx_byte;
    wire       spi_rx_valid;
    reg  [7:0] spi_tx_byte;

    spi_slave u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .spi_clk  (spi_sck),
        .spi_mosi (spi_mosi),
        .spi_cs_n (spi_cs_n),
        .spi_miso (spi_miso),
        .rx_byte  (spi_rx_byte),
        .rx_valid (spi_rx_valid),
        .tx_byte  (spi_tx_byte)
    );

    // ================================================================
    // Operand Registers
    // ================================================================
    reg signed [15:0] reg_a;   // Q8.8 operand A
    reg signed [15:0] reg_b;   // Q8.8 operand B
    reg signed [15:0] reg_c;   // Q8.8 operand C
    reg signed [15:0] result;  // Q8.8 last result

    // ================================================================
    // Multiplier (combinational)
    // ================================================================
    reg  signed [15:0] mul_a_in, mul_b_in;
    wire signed [15:0] mul_result;
    wire               mul_ovf;

    fixedpoint_mul u_mul (
        .a      (mul_a_in),
        .b      (mul_b_in),
        .result (mul_result),
        .ovf    (mul_ovf)
    );

    // ================================================================
    // Divider (iterative, 17 clk cycles)
    // ================================================================
    reg  signed [15:0] div_dividend_in;
    reg  signed [15:0] div_divisor_in;
    reg                div_start;
    wire signed [15:0] div_result;
    wire               div_done;
    wire               div_dbz;
    wire               div_ovf;

    fixedpoint_div u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend_in),
        .divisor    (div_divisor_in),
        .result     (div_result),
        .done       (div_done),
        .div_by_zero(div_dbz),
        .ovf        (div_ovf)
    );

    // ================================================================
    // Command Decoder State Machine
    // ================================================================
    // States
    localparam S_IDLE        = 4'd0;
    localparam S_LOAD_A_HI   = 4'd1;
    localparam S_LOAD_A_LO   = 4'd2;
    localparam S_LOAD_B_HI   = 4'd3;
    localparam S_LOAD_B_LO   = 4'd4;
    localparam S_LOAD_C_HI   = 4'd5;
    localparam S_LOAD_C_LO   = 4'd6;
    localparam S_WAIT_DIV    = 4'd7;
    localparam S_MACDIV_PREP = 4'd8;
    localparam S_MACDIV_WAIT = 4'd9;

    // Commands
    localparam CMD_LOAD_A    = 8'h10;
    localparam CMD_LOAD_B    = 8'h11;
    localparam CMD_LOAD_C    = 8'h12;
    localparam CMD_MUL       = 8'h20;
    localparam CMD_DIV       = 8'h21;
    localparam CMD_MACDIV    = 8'h22;
    localparam CMD_READ_RES  = 8'h30;
    localparam CMD_READ_STAT = 8'h31;

    reg [3:0] state;
    reg       busy_r;
    reg       done_r;
    reg       ovf_r;
    reg       dbz_r;
    reg [7:0] tmp_hi;          // temporary high byte when loading 16-bit values
    reg       read_hi_sent;    // true after high byte placed in tx_byte

    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            reg_a        <= 16'h0000;
            reg_b        <= 16'h0100;   // 1.0 in Q8.8
            reg_c        <= 16'h0000;
            result       <= 16'h0000;
            busy_r       <= 1'b0;
            done_r       <= 1'b0;
            ovf_r        <= 1'b0;
            dbz_r        <= 1'b0;
            div_start    <= 1'b0;
            mul_a_in     <= 16'h0000;
            mul_b_in     <= 16'h0000;
            spi_tx_byte  <= 8'h00;
            tmp_hi       <= 8'h00;
            read_hi_sent <= 1'b0;
        end else begin
            div_start <= 1'b0;  // default: not starting a division
            done_r    <= 1'b0;  // default: not done

            // ── Capture division result when ready ──────────────────
            if (div_done && (state == S_WAIT_DIV || state == S_MACDIV_WAIT)) begin
                result <= div_result;
                ovf_r  <= ovf_r | div_ovf;
                dbz_r  <= div_dbz;
                busy_r <= 1'b0;
                done_r <= 1'b1;
                state  <= S_IDLE;
            end

            // ── SPI byte received ───────────────────────────────────
            if (spi_rx_valid) begin
                case (state)

                    S_IDLE: begin
                        // First byte is always a command
                        ovf_r <= 1'b0;  // clear flags on new command
                        dbz_r <= 1'b0;

                        case (spi_rx_byte)
                            CMD_LOAD_A:    state <= S_LOAD_A_HI;
                            CMD_LOAD_B:    state <= S_LOAD_B_HI;
                            CMD_LOAD_C:    state <= S_LOAD_C_HI;

                            CMD_MUL: begin
                                // Combinational multiply: result is ready immediately
                                mul_a_in <= reg_a;
                                mul_b_in <= reg_b;
                                // mul_result updates combinationally;
                                // register it next cycle
                                state    <= S_IDLE;
                                done_r   <= 1'b1;
                                // Note: result registered on next clk below
                            end

                            CMD_DIV: begin
                                div_dividend_in <= reg_a;
                                div_divisor_in  <= reg_b;
                                div_start       <= 1'b1;
                                busy_r          <= 1'b1;
                                state           <= S_WAIT_DIV;
                            end

                            CMD_MACDIV: begin
                                // K = A / (A + C)   [scalar Kalman gain, H=1]
                                // Step 1: compute (A + C) using the multiplier as an adder
                                // We use the adder path: divisor = A + C
                                // dividend = A
                                // We can compute A+C with just addition (no multiplier needed).
                                // Then divide A by (A+C).
                                div_dividend_in <= reg_a;
                                div_divisor_in  <= reg_a + reg_c;  // P + R
                                div_start       <= 1'b1;
                                busy_r          <= 1'b1;
                                state           <= S_WAIT_DIV;
                            end

                            CMD_READ_RES: begin
                                // Prepare high byte for clocking out
                                spi_tx_byte  <= result[15:8];
                                read_hi_sent <= 1'b1;
                                state        <= S_IDLE;
                                // Low byte will be sent on the next rx_valid
                                // because spi_slave pre-loads tx_byte at CS low
                                // and we update it mid-transfer here.
                                // A cleaner approach: RP2040 does two separate
                                // CS transactions for HI and LO bytes.
                                // See protocol note in README.
                            end

                            CMD_READ_STAT: begin
                                spi_tx_byte <= {ovf_r, dbz_r, busy_r, done_r, 4'b0000};
                                state       <= S_IDLE;
                            end

                            default: state <= S_IDLE;
                        endcase
                    end

                    // ── Multi-byte operand loading ──────────────────
                    S_LOAD_A_HI: begin tmp_hi <= spi_rx_byte; state <= S_LOAD_A_LO; end
                    S_LOAD_A_LO: begin reg_a  <= {tmp_hi, spi_rx_byte}; state <= S_IDLE; end

                    S_LOAD_B_HI: begin tmp_hi <= spi_rx_byte; state <= S_LOAD_B_LO; end
                    S_LOAD_B_LO: begin reg_b  <= {tmp_hi, spi_rx_byte}; state <= S_IDLE; end

                    S_LOAD_C_HI: begin tmp_hi <= spi_rx_byte; state <= S_LOAD_C_LO; end
                    S_LOAD_C_LO: begin reg_c  <= {tmp_hi, spi_rx_byte}; state <= S_IDLE; end

                    // ── During division, ignore incoming bytes ──────
                    S_WAIT_DIV, S_MACDIV_WAIT:;

                    default: state <= S_IDLE;
                endcase
            end

            // ── Register multiply result (combinational path) ───────
            // This runs the clock after CMD_MUL is decoded
            if (done_r && state == S_IDLE && !div_done) begin
                result <= mul_result;
                ovf_r  <= mul_ovf;
            end

            // ── READ_RES second byte: drive result low byte ─────────
            if (read_hi_sent && spi_rx_valid) begin
                spi_tx_byte  <= result[7:0];
                read_hi_sent <= 1'b0;
            end
        end
    end

    // ================================================================
    // Outputs
    // ================================================================
    assign uo_out  = result[15:8];   // high byte of result on dedicated outputs
    assign busy_out = busy_r;
    assign done_out = done_r;
    assign ovf_out  = ovf_r;
    assign dbz_out  = dbz_r;

endmodule
