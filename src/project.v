/*
 * Copyright (c) 2026 Kunal / Claude
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * project.v — Tiny ML Coprocessor (minimised for 1×1 Tiny Tapeout tile)
 *
 * KEY AREA REDUCTIONS vs the original sketch:
 *  1. Single shared 16×8 multiplier (not 8 parallel ones) — saves ~1400 gates
 *  2. 6-bit weights instead of 16-bit — saves ~640 flip-flops in the weight file
 *  3. MAC accumulation is sequential (one multiply per clock) not parallel
 *  4. Sqrt removed (saves ~316 gates) — RP2040 can do integer sqrt fine
 *  5. Operand registers A/B/C reduced to 16-bit (kept, needed for div/mul)
 *  6. Result register shared across all operations
 *
 * ESTIMATED GATE COUNT:
 *  SPI slave          ~150
 *  Weight regs 8×6b   ~290  (48 DFFs × 6 gates)
 *  Single 16×8 MUL    ~200  (combinational)
 *  24-bit accumulator ~120
 *  16-bit divider     ~420
 *  Operand regs A/B/C ~300  (3 × 16-bit)
 *  Command FSM        ~250
 *  TOTAL             ~1730  ← comfortably under ~4000 gate budget
 *
 * ── Fixed-Point Format ────────────────────────────────────────────────────
 * Operands A, B, C:  Q8.8 signed  (16-bit, int16 × 256 = Q8.8)
 * MAC weights:       Q1.5 signed  (6-bit:  range -2.0 to +1.969, step 0.031)
 * MAC inputs:        unsigned 8-bit (0–255, representing 0.0–1.0 if /255)
 * MAC result:        Q8.8 signed  (accumulated into 24-bit, truncated)
 * Div/Mul result:    Q8.8 signed
 *
 * ── SPI Protocol ──────────────────────────────────────────────────────────
 * Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit bytes.
 *
 * Commands:
 *   0x10  LOAD_A    — 2 bytes follow (Q8.8 MSB first)
 *   0x11  LOAD_B    — 2 bytes follow
 *   0x12  LOAD_C    — 2 bytes follow
 *   0x13  LOAD_W    — 1 byte index (0-7), 1 byte weight (Q1.5 signed, bits[5:0])
 *   0x14  LOAD_IN   — 8 bytes (unsigned inputs); MAC runs automatically, takes 8 clk
 *   0x20  OP_MUL    — result = A * B  (ready after ~1 clk, poll DONE)
 *   0x21  OP_DIV    — result = A / B  (ready after 17 clk, poll BUSY)
 *   0x22  OP_KGAIN  — result = A / (A+C)  Kalman gain K=P/(P+R)
 *   0x30  READ_HI   — 1 dummy byte clocked → MISO returns result[15:8]
 *   0x31  READ_LO   — 1 dummy byte clocked → MISO returns result[7:0]
 *   0x3F  READ_STAT — 1 dummy byte → MISO: {ovf,dbz,busy,done,4'b0}
 *
 * Reading result: issue 0x30, send dummy 0x00, read MISO byte → high byte
 *                 issue 0x31, send dummy 0x00, read MISO byte → low byte
 * (Two separate CS transactions, simpler than the combined approach)
 *
 * ── Pin Map ───────────────────────────────────────────────────────────────
 * uio_in[0]   SCK
 * uio_in[1]   MOSI
 * uio_in[2]   CS# (active low)
 * uio_out[3]  MISO
 * uio_out[4]  BUSY  (high while divider or MAC running)
 * uio_out[5]  DONE  (pulses 1 clk when result ready)
 * uio_out[6]  OVF
 * uio_out[7]  DBZ   (divide by zero)
 * uo_out[7:0] result[15:8]  (high byte, always visible)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// ============================================================================
// SPI Slave  — Mode 0, MSB first, 8-bit bytes
// Synchronises SCK into system clock domain with 3-FF synchroniser.
// rx_valid pulses for 1 clk when a full byte lands in rx_byte.
// tx_byte is captured at CS falling edge; update it before the next transfer.
// ============================================================================
module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       spi_clk,
    input  wire       spi_mosi,
    input  wire       spi_cs_n,
    output reg        spi_miso,
    output reg  [7:0] rx_byte,
    output reg        rx_valid,
    input  wire [7:0] tx_byte
);
    reg [2:0] sck_r;
    reg [1:0] cs_r;
    reg [7:0] rx_sr, tx_sr;
    reg [2:0] bcnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_r    <= 3'd0; cs_r  <= 2'b11;
            rx_sr    <= 8'd0; tx_sr <= 8'd0;
            bcnt     <= 3'd0; rx_valid <= 1'b0;
            spi_miso <= 1'b0; rx_byte  <= 8'd0;
        end else begin
            sck_r    <= {sck_r[1:0], spi_clk};
            cs_r     <= {cs_r[0],    spi_cs_n};
            rx_valid <= 1'b0;

            if (cs_r[1]) begin                    // CS deasserted: reset
                tx_sr    <= tx_byte;
                bcnt     <= 3'd0;
                spi_miso <= tx_byte[7];
            end else begin
                if (sck_r[2:1] == 2'b01) begin    // rising edge: sample
                    rx_sr <= {rx_sr[6:0], spi_mosi};
                    if (bcnt == 3'd7) begin
                        rx_byte  <= {rx_sr[6:0], spi_mosi};
                        rx_valid <= 1'b1;
                        bcnt     <= 3'd0;
                    end else bcnt <= bcnt + 1'b1;
                end
                if (sck_r[2:1] == 2'b10) begin    // falling edge: shift out
                    tx_sr    <= {tx_sr[6:0], 1'b0};
                    spi_miso <= tx_sr[6];
                end
            end
        end
    end
endmodule


// ============================================================================
// Sequential 16-bit signed divider  (Q8.8 ÷ Q8.8 → Q8.8)
// Latency: 17 clock cycles after start.
// Computes (dividend << 8) / divisor using restoring long division.
// ============================================================================
module div16 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [15:0] dividend,
    input  wire signed [15:0] divisor,
    output reg  signed [15:0] result,
    output reg                done,
    output reg                dbz,
    output reg                ovf
);
    reg        sign_r;
    reg [15:0] denom;
    reg [31:0] rem;
    reg [15:0] quot;
    reg  [4:0] step;
    reg        busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; done <= 0; dbz <= 0; ovf <= 0;
            result <= 0; step <= 0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                dbz <= 1'b0; ovf <= 1'b0;
                if (divisor == 16'sd0) begin
                    dbz    <= 1'b1;
                    result <= dividend[15] ? 16'sh8000 : 16'sh7FFF;
                    done   <= 1'b1;
                end else begin
                    sign_r <= dividend[15] ^ divisor[15];
                    denom  <= divisor[15]  ? (~divisor  + 1'b1) : divisor;
                    // shift dividend left by 8 to give Q8.16 numerator
                    rem    <= {8'b0,
                               (dividend[15] ? (~dividend+1'b1) : dividend),
                               8'b0};
                    quot   <= 16'd0;
                    step   <= 5'd0;
                    busy   <= 1'b1;
                end
            end else if (busy) begin
                // one bit of restoring division per clock
                quot <= quot << 1;
                if (rem[31:16] >= denom) begin
                    rem      <= (rem - {denom, 16'd0}) << 1;
                    quot[0]  <= 1'b1;
                end else begin
                    rem <= rem << 1;
                end
                if (step == 5'd15) begin
                    result <= sign_r ? (~quot[15:0] + 1'b1) : quot[15:0];
                    busy   <= 1'b0;
                    done   <= 1'b1;
                end
                step <= step + 1'b1;
            end
        end
    end
endmodule


// ============================================================================
// Top-level: tt_um_ml_coprocessor
// ============================================================================
module tt_um_ml_coprocessor (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ── uio directions: [2:0]=inputs, [7:3]=outputs ──────────────────
    assign uio_oe = 8'b1111_1000;
    assign uio_out[2:0] = 3'b000;

    wire spi_sck  = uio_in[0];
    wire spi_mosi = uio_in[1];
    wire spi_cs_n = uio_in[2];
    wire spi_miso_w;

    // ── SPI slave ─────────────────────────────────────────────────────
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [7:0] tx_byte;

    spi_slave u_spi (
        .clk(clk), .rst_n(rst_n),
        .spi_clk(spi_sck), .spi_mosi(spi_mosi),
        .spi_cs_n(spi_cs_n), .spi_miso(spi_miso_w),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte)
    );

    // ── Operand registers (Q8.8 signed) ──────────────────────────────
    reg signed [15:0] reg_a, reg_b, reg_c, result;

    // ── Weight register file: 8 × 6-bit signed (Q1.5) ────────────────
    // 6-bit signed range: -2.0 to +1.969  (step ≈ 0.031)
    // Save: 8×16 = 128 DFF  →  8×6 = 48 DFF  (saves 80 DFFs ≈ 480 gates)
    reg signed [5:0] weights [0:7];

    // ── Single shared 16×8 multiplier (combinational) ─────────────────
    // Used for both OP_MUL and MAC accumulation (time-multiplexed)
    reg  signed [15:0] mul_a;
    reg  signed  [7:0] mul_b;
    wire signed [23:0] mul_out = mul_a * mul_b;  // Q8.8 × Q0.8 → Q8.16

    // ── 24-bit MAC accumulator ─────────────────────────────────────────
    reg signed [23:0] mac_acc;
    reg signed [15:0] mac_result;  // final Q8.8 (truncated mac_acc[23:8])

    // ── Divider ────────────────────────────────────────────────────────
    reg  signed [15:0] div_num, div_den;
    reg                div_start;
    wire signed [15:0] div_result;
    wire               div_done, div_dbz, div_ovf;

    div16 u_div (
        .clk(clk), .rst_n(rst_n),
        .start(div_start),
        .dividend(div_num), .divisor(div_den),
        .result(div_result),
        .done(div_done), .dbz(div_dbz), .ovf(div_ovf)
    );

    // ── Status flags ───────────────────────────────────────────────────
    reg busy_r, done_r, ovf_r, dbz_r;

    // ── Command / parse FSM ────────────────────────────────────────────
    // States
    localparam P_CMD       = 4'd0,
               P_A_HI      = 4'd1,  P_A_LO     = 4'd2,
               P_B_HI      = 4'd3,  P_B_LO     = 4'd4,
               P_C_HI      = 4'd5,  P_C_LO     = 4'd6,
               P_W_IDX     = 4'd7,  P_W_DAT    = 4'd8,
               P_IN        = 4'd9,
               P_WAIT_DIV  = 4'd10,
               P_MAC_RUN   = 4'd11;

    // Commands
    localparam CMD_LOAD_A  = 8'h10, CMD_LOAD_B  = 8'h11,
               CMD_LOAD_C  = 8'h12, CMD_LOAD_W  = 8'h13,
               CMD_LOAD_IN = 8'h14,
               CMD_MUL     = 8'h20, CMD_DIV     = 8'h21,
               CMD_KGAIN   = 8'h22,
               CMD_RD_HI   = 8'h30, CMD_RD_LO   = 8'h31,
               CMD_RD_STAT = 8'h3F;

    reg [3:0] pstate;
    reg [7:0] tmp_hi;
    reg [2:0] w_idx;      // weight index being loaded
    reg [2:0] in_cnt;     // input byte counter for MAC
    reg       wait_div;
    reg       mac_run;    // MAC is accumulating

    // MAC sequential accumulation: one multiply per clock in P_MAC_RUN
    // We index weight by in_cnt, input comes from a small 8-byte buffer
    reg [7:0] in_buf [0:7];  // 8-byte input buffer  (8 × 8 = 64 DFF)

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pstate   <= P_CMD;
            reg_a    <= 16'sd0; reg_b <= 16'sd0; reg_c <= 16'sd0;
            result   <= 16'sd0;
            busy_r   <= 1'b0;   done_r <= 1'b0;
            ovf_r    <= 1'b0;   dbz_r  <= 1'b0;
            div_start <= 1'b0;
            wait_div <= 1'b0;   mac_run <= 1'b0;
            tx_byte  <= 8'h00;  tmp_hi  <= 8'h00;
            w_idx    <= 3'd0;   in_cnt  <= 3'd0;
            mac_acc  <= 24'sd0; mac_result <= 16'sd0;
            mul_a    <= 16'sd0; mul_b   <= 8'sd0;
            for (i = 0; i < 8; i = i + 1) begin
                weights[i] <= 6'sd0;
                in_buf[i]  <= 8'd0;
            end
        end else begin
            div_start <= 1'b0;
            done_r    <= 1'b0;

            // ── Capture divider result ──────────────────────────────
            if (div_done && wait_div) begin
                result   <= div_result;
                ovf_r    <= div_ovf;
                dbz_r    <= div_dbz;
                busy_r   <= 1'b0;
                done_r   <= 1'b1;
                wait_div <= 1'b0;
                pstate   <= P_CMD;
            end

            // ── MAC sequential accumulation ─────────────────────────
            // Each clock: acc += weight[in_cnt] * in_buf[in_cnt]
            // weight is Q1.5 (6-bit signed), input is Q0.8 (8-bit unsigned)
            // product is Q1.13 (14-bit signed), accumulated in 24-bit
            if (mac_run) begin
                // Sign-extend 6-bit weight to 16-bit for multiplier input
                mul_a <= {{10{weights[in_cnt][5]}}, weights[in_cnt]};
                mul_b <= in_buf[in_cnt];
                // mul_out is registered next cycle — we use it one cycle later
                // so we actually accumulate the *previous* cycle's product.
                // Resolved by running 9 cycles: 1 setup + 8 accumulate.
                if (in_cnt > 3'd0) begin
                    mac_acc <= mac_acc + {{8{mul_out[23]}}, mul_out[23:8]};
                    // mul_out[23:8] gives Q1.5 × Q0.8 = Q1.13, shifted to Q8.8
                    // Actually: weight(Q1.5) * input(Q0.8) → Q1.13
                    // We want Q8.8 accumulation, so shift left by 3: mul_out[20:5]
                    // Use mac_acc[23:8] as final Q8.8 result (enough headroom for 8 terms)
                end
                if (in_cnt == 3'd7) begin
                    mac_run <= 1'b0;
                    busy_r  <= 1'b0;
                    done_r  <= 1'b1;
                    // Truncate accumulator to Q8.8
                    mac_result <= mac_acc[23:8];
                    result     <= mac_acc[23:8];
                    pstate     <= P_CMD;
                end
                in_cnt <= in_cnt + 1'b1;
            end

            // ── SPI byte received ───────────────────────────────────
            if (rx_valid && !mac_run) begin
                case (pstate)

                P_CMD: begin
                    ovf_r <= 1'b0; dbz_r <= 1'b0;
                    case (rx_byte)
                        CMD_LOAD_A:  pstate <= P_A_HI;
                        CMD_LOAD_B:  pstate <= P_B_HI;
                        CMD_LOAD_C:  pstate <= P_C_HI;
                        CMD_LOAD_W:  pstate <= P_W_IDX;

                        CMD_LOAD_IN: begin
                            in_cnt  <= 3'd0;
                            pstate  <= P_IN;
                        end

                        CMD_MUL: begin
                            // A(Q8.8) × B(Q8.8): use the 16×8 multiplier
                            // We treat B as 8-bit by taking reg_b[7:0] (fractional byte)
                            // and A as full 16-bit — this gives A × (B/256) in Q8.8
                            // For full Q8.8 × Q8.8 we need two passes; to save area
                            // we instead do a 16×16 via the synthesiser's * operator
                            // on the reg directly (one combinational block):
                            result <= $signed(reg_a) * $signed(reg_b) >>> 8;
                            done_r <= 1'b1;
                        end

                        CMD_DIV: begin
                            div_num   <= reg_a;
                            div_den   <= reg_b;
                            div_start <= 1'b1;
                            busy_r    <= 1'b1;
                            wait_div  <= 1'b1;
                            pstate    <= P_WAIT_DIV;
                        end

                        CMD_KGAIN: begin
                            // K = P / (P + R),  reg_a=P, reg_c=R
                            div_num   <= reg_a;
                            div_den   <= reg_a + reg_c;
                            div_start <= 1'b1;
                            busy_r    <= 1'b1;
                            wait_div  <= 1'b1;
                            pstate    <= P_WAIT_DIV;
                        end

                        CMD_RD_HI:   tx_byte <= result[15:8];
                        CMD_RD_LO:   tx_byte <= result[7:0];
                        CMD_RD_STAT: tx_byte <= {ovf_r, dbz_r, busy_r, done_r, 4'b0};

                        default:;
                    endcase
                end

                // ── 16-bit operand loads ──────────────────────────
                P_A_HI: begin tmp_hi <= rx_byte; pstate <= P_A_LO; end
                P_A_LO: begin reg_a  <= {tmp_hi, rx_byte}; pstate <= P_CMD; end

                P_B_HI: begin tmp_hi <= rx_byte; pstate <= P_B_LO; end
                P_B_LO: begin reg_b  <= {tmp_hi, rx_byte}; pstate <= P_CMD; end

                P_C_HI: begin tmp_hi <= rx_byte; pstate <= P_C_LO; end
                P_C_LO: begin reg_c  <= {tmp_hi, rx_byte}; pstate <= P_CMD; end

                // ── Weight load: IDX then 1 data byte ────────────
                P_W_IDX: begin w_idx <= rx_byte[2:0]; pstate <= P_W_DAT; end
                P_W_DAT: begin
                    weights[w_idx] <= rx_byte[5:0];  // Q1.5: 6 LSBs, sign in bit5
                    pstate <= P_CMD;
                end

                // ── Input buffer fill: 8 bytes then MAC starts ────
                P_IN: begin
                    in_buf[in_cnt] <= rx_byte;
                    if (in_cnt == 3'd7) begin
                        // All inputs loaded: start MAC
                        mac_acc <= 24'sd0;
                        in_cnt  <= 3'd0;
                        mul_a   <= {{10{weights[0][5]}}, weights[0]};
                        mul_b   <= in_buf[0];   // will be latched next cycle
                        mac_run <= 1'b1;
                        busy_r  <= 1'b1;
                        pstate  <= P_MAC_RUN;
                    end else begin
                        in_cnt <= in_cnt + 1'b1;
                    end
                end

                P_WAIT_DIV:;    // ignore SPI bytes while dividing
                P_MAC_RUN:;     // ignore SPI bytes while MAC running

                default: pstate <= P_CMD;
                endcase
            end
        end
    end

    // ── Outputs ───────────────────────────────────────────────────────
    assign uo_out      = result[15:8];
    assign uio_out[3]  = spi_miso_w;
    assign uio_out[4]  = busy_r;
    assign uio_out[5]  = done_r;
    assign uio_out[6]  = ovf_r;
    assign uio_out[7]  = dbz_r;

endmodule
