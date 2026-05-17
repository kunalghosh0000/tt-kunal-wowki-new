/*
 * Copyright (c) 2026 Kunal / Claude
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * project.v — Tiny ML Coprocessor  (aggressively minimised, ~230 DFF target)
 *
 * Changes from previous version to hit <100% utilisation:
 *
 *  1. in_buf[8×8] REMOVED (64 DFF saved)
 *     MAC inputs streamed one byte at a time via CMD 0x14 (start) + 0x15 (byte).
 *     Accumulation on the fly — no buffer needed.
 *
 *  2. mul_a / mul_b staging registers REMOVED (24 DFF saved)
 *     Multiplier reads weight register and rx_byte combinationally.
 *
 *  3. mac_result register REMOVED (16 DFF saved)
 *     MAC writes directly into shared `result`.
 *
 *  4. reg_c REMOVED (16 DFF saved)
 *     Kalman gain CMD removed. RP2040 computes (P+R) in one instruction
 *     and loads it into reg_b. Chip just does reg_a / reg_b.
 *
 *  5. Divider rem[] cut 32→24 bits (8 DFF saved)
 *
 *  6. mac_acc cut 24→20 bits (4 DFF saved)
 *
 *  7. Weights cut 6→5 bits Q1.4 (8 DFF saved)
 *
 *  8. Parse FSM cut from 4-bit/12 states to 3-bit/7 states (saves decode logic)
 *     MAC bytes intercepted by mac_active flag before state decode.
 *
 * ── Fixed-Point Format ───────────────────────────────────────────────────
 * A, B, result:  Q8.8 signed 16-bit
 * MAC weights:   Q1.4 signed 5-bit  (range -1.0 to +0.9375, step 0.0625)
 * MAC inputs:    unsigned 8-bit (0-255)
 * MAC result:    Q8.8 in result register
 *
 * ── SPI Protocol ─────────────────────────────────────────────────────────
 * Mode 0, MSB first, 8-bit bytes.
 *
 *  0x10 HI LO   → reg_a = {HI,LO}
 *  0x11 HI LO   → reg_b = {HI,LO}
 *  0x13 IDX DAT → weights[IDX] = DAT[4:0]
 *  0x14         → clear mac_acc, mac_cnt=0, assert BUSY
 *  0x15 DAT     → mac_acc += weights[mac_cnt]*DAT; mac_cnt++
 *                 (send exactly 8 times after 0x14; result ready when BUSY low)
 *  0x20         → result = reg_a * reg_b (Q8.8, combinational)
 *  0x21         → result = reg_a / reg_b (17 clk, poll BUSY)
 *  0x30 DUM     → MISO = result[15:8]
 *  0x31 DUM     → MISO = result[7:0]
 *  0x3F DUM     → MISO = {ovf,dbz,busy,done,4'b0}
 *
 * Kalman gain K = P/(P+R):   RP2040 computes (P+R), loads into B, P into A,
 *                             issues 0x21 (DIV). Chip returns K in result.
 *
 * ── Pins ─────────────────────────────────────────────────────────────────
 * uio_in[0]=SCK  [1]=MOSI  [2]=CS#
 * uio_out[3]=MISO [4]=BUSY [5]=DONE [6]=OVF [7]=DBZ
 * uo_out[7:0] = result[15:8]
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
`default_nettype none
 
// ============================================================================
// SPI Slave — Mode 0, MSB first
// ============================================================================
module spi_slave (
    input  wire       clk, rst_n,
    input  wire       spi_clk, spi_mosi, spi_cs_n,
    output reg        spi_miso,
    output reg  [7:0] rx_byte,
    output reg        rx_valid,
    input  wire [7:0] tx_byte
);
    reg [2:0] sck_r;
    reg       cs_r;
    reg [7:0] rx_sr, tx_sr;
    reg [2:0] bcnt;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_r<=3'd0; cs_r<=1'b1; rx_sr<=8'd0; tx_sr<=8'd0;
            bcnt<=3'd0; rx_valid<=1'b0; spi_miso<=1'b0; rx_byte<=8'd0;
        end else begin
            sck_r    <= {sck_r[1:0], spi_clk};
            cs_r     <= spi_cs_n;
            rx_valid <= 1'b0;
            if (cs_r) begin
                tx_sr <= tx_byte; bcnt <= 3'd0; spi_miso <= tx_byte[7];
            end else begin
                if (sck_r[2:1] == 2'b01) begin
                    rx_sr <= {rx_sr[6:0], spi_mosi};
                    if (bcnt == 3'd7) begin
                        rx_byte  <= {rx_sr[6:0], spi_mosi};
                        rx_valid <= 1'b1;
                        bcnt     <= 3'd0;
                    end else bcnt <= bcnt + 3'd1;
                end
                if (sck_r[2:1] == 2'b10) begin
                    tx_sr    <= {tx_sr[6:0], 1'b0};
                    spi_miso <= tx_sr[6];
                end
            end
        end
    end
endmodule
 
 
// ============================================================================
// 16-bit signed divider — 16 cycles latency
// Computes (dividend << 8) / divisor → Q8.8 result
// 24-bit remainder (down from 32): saves 8 DFF
// ============================================================================
module div16 (
    input  wire        clk, rst_n, start,
    input  wire signed [15:0] dividend, divisor,
    output reg  signed [15:0] result,
    output reg                done, dbz
);
    reg        sgn;
    reg [15:0] den;
    reg [23:0] rem;
    reg [15:0] quot;
    reg  [3:0] step;
    reg        busy;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=1'b0; done<=1'b0; dbz<=1'b0; result<=16'sd0; step<=4'd0;
            sgn<=1'b0; den<=16'd0; rem<=24'd0; quot<=16'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                dbz <= 1'b0;
                if (divisor == 16'sd0) begin
                    dbz    <= 1'b1;
                    result <= dividend[15] ? 16'sh8000 : 16'sh7FFF;
                    done   <= 1'b1;
                end else begin
                    sgn  <= dividend[15] ^ divisor[15];
                    den  <= divisor[15]  ? (~divisor  + 1'b1) : divisor;
                    rem  <= {8'b0, dividend[15] ? (~dividend + 1'b1) : dividend};
                    quot <= 16'd0;
                    step <= 4'd0;
                    busy <= 1'b1;
                end
            end else if (busy) begin
                if (rem[23:8] >= den) begin
                    rem  <= {(rem[23:8] - den), rem[7:0], 1'b1};
                    quot <= {quot[14:0], 1'b1};
                end else begin
                    rem  <= {rem[22:0], 1'b0};
                    quot <= {quot[14:0], 1'b0};
                end
                if (step == 4'd15) begin
                    result <= sgn ? (~quot + 1'b1) : quot;
                    busy <= 1'b0; done <= 1'b1;
                end
                step <= step + 4'd1;
            end
        end
    end
endmodule
 
 
// ============================================================================
// Top-level
// ============================================================================
module tt_um_ml_coprocessor (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena, clk, rst_n
);
    assign uio_oe       = 8'b1111_1000;
    assign uio_out[2:0] = 3'b000;
 
    wire       spi_miso_w;
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [7:0] tx_byte;
 
    spi_slave u_spi (
        .clk(clk), .rst_n(rst_n),
        .spi_clk(uio_in[0]), .spi_mosi(uio_in[1]), .spi_cs_n(uio_in[2]),
        .spi_miso(spi_miso_w),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .tx_byte(tx_byte)
    );
 
    // ── Operand registers ─────────────────────────────────────────────
    reg signed [15:0] reg_a, reg_b, result;
 
    // ── Weight file: 8 × 5-bit signed Q1.4 ───────────────────────────
    reg signed [4:0] weights [0:7];
 
    // ── Divider ───────────────────────────────────────────────────────
    reg         div_start;
    wire signed [15:0] div_result;
    wire               div_done, div_dbz;
 
    div16 u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .dividend(reg_a), .divisor(reg_b),
        .result(div_result), .done(div_done), .dbz(div_dbz)
    );
 
    // ── MAC (no buffer: accumulate rx_byte on the fly) ────────────────
    // Combinational product: weight[mac_cnt] (Q1.4, 5-bit) × rx_byte (Q0.8, 8-bit)
    // = Q1.12 signed, 13 bits. Accumulated in 20-bit signed.
    reg [2:0]  mac_cnt;
    reg signed [19:0] mac_acc;
    reg        mac_active;
 
    // sign-extend 5-bit weight to 13 bits for multiply
    wire signed [12:0] mac_prod =
        $signed({{8{weights[mac_cnt][4]}}, weights[mac_cnt]}) *
        $signed({1'b0, rx_byte});   // 13s × 9u → 13-bit result (13+9=22, but values fit 13)
 
    // ── Status ────────────────────────────────────────────────────────
    reg busy_r, done_r, ovf_r, dbz_r;
 
    // ── Parse FSM (3-bit, 7 states) ───────────────────────────────────
    localparam [2:0]
        P_CMD    = 3'd0,
        P_A_HI   = 3'd1, P_A_LO = 3'd2,
        P_B_HI   = 3'd3, P_B_LO = 3'd4,
        P_W_IDX  = 3'd5, P_W_DAT= 3'd6;
 
    reg [2:0] pstate;
    reg [7:0] tmp_hi;
    reg [2:0] w_idx;
 
    integer k;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pstate    <= P_CMD;
            reg_a     <= 16'sd0; reg_b   <= 16'sd0; result  <= 16'sd0;
            busy_r    <= 1'b0;   done_r  <= 1'b0;
            ovf_r     <= 1'b0;   dbz_r   <= 1'b0;
            div_start <= 1'b0;
            mac_cnt   <= 3'd0;   mac_acc <= 20'sd0; mac_active <= 1'b0;
            tx_byte   <= 8'h00;  tmp_hi  <= 8'h00;  w_idx <= 3'd0;
            for (k = 0; k < 8; k = k+1) weights[k] <= 5'sd0;
        end else begin
            div_start <= 1'b0;
            done_r    <= 1'b0;
 
            // ── Divider completion ─────────────────────────────────
            if (div_done) begin
                result <= div_result;
                dbz_r  <= div_dbz;
                busy_r <= 1'b0;
                done_r <= 1'b1;
            end
 
            // ── SPI byte received ──────────────────────────────────
            if (rx_valid) begin
 
                // MAC byte intercept (cmd 0x15 handled here, no state needed)
                if (mac_active) begin
                    mac_acc <= mac_acc + {{7{mac_prod[12]}}, mac_prod};
                    if (mac_cnt == 3'd7) begin
                        // Shift right by 4: Q1.12 → Q1.8, pack into Q8.8 result
                        result     <= mac_acc[19:4];
                        mac_active <= 1'b0;
                        busy_r     <= 1'b0;
                        done_r     <= 1'b1;
                    end
                    mac_cnt <= mac_cnt + 3'd1;
 
                end else begin
 
                case (pstate)
 
                P_CMD: begin
                    ovf_r <= 1'b0; dbz_r <= 1'b0;
                    case (rx_byte)
                        8'h10: pstate <= P_A_HI;
                        8'h11: pstate <= P_B_HI;
                        8'h13: pstate <= P_W_IDX;
 
                        8'h14: begin          // MAC start
                            mac_acc    <= 20'sd0;
                            mac_cnt    <= 3'd0;
                            mac_active <= 1'b1;
                            busy_r     <= 1'b1;
                        end
 
                        8'h20: begin          // MUL
                            result <= ($signed(reg_a) * $signed(reg_b)) >>> 8;
                            done_r <= 1'b1;
                        end
 
                        8'h21: begin          // DIV (uses reg_a/reg_b directly)
                            div_start <= 1'b1;
                            busy_r    <= 1'b1;
                        end
 
                        8'h30: tx_byte <= result[15:8];
                        8'h31: tx_byte <= result[7:0];
                        8'h3F: tx_byte <= {ovf_r, dbz_r, busy_r, done_r, 4'b0000};
                        default:;
                    endcase
                end
 
                P_A_HI: begin tmp_hi <= rx_byte; pstate <= P_A_LO; end
                P_A_LO: begin reg_a  <= {tmp_hi, rx_byte}; pstate <= P_CMD; end
 
                P_B_HI: begin tmp_hi <= rx_byte; pstate <= P_B_LO; end
                P_B_LO: begin reg_b  <= {tmp_hi, rx_byte}; pstate <= P_CMD; end
 
                P_W_IDX: begin w_idx  <= rx_byte[2:0]; pstate <= P_W_DAT; end
                P_W_DAT: begin
                    weights[w_idx] <= rx_byte[4:0];
                    pstate <= P_CMD;
                end
 
                default: pstate <= P_CMD;
                endcase
                end // !mac_active
            end
        end
    end
 
    assign uo_out     = result[15:8];
    assign uio_out[3] = spi_miso_w;
    assign uio_out[4] = busy_r;
    assign uio_out[5] = done_r;
    assign uio_out[6] = ovf_r;
    assign uio_out[7] = dbz_r;
 
endmodule
 
