/*
 * Copyright (c) 2026 Kunal / Claude
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * tt_um_ml_coprocessor.v — Tiny ML Coprocessor (Project D)
 *
 * Tiny Tapeout top-level.
 *
 * Combines:
 *  - 8-element signed Q8.8 MAC array (dot product accelerator)
 *  - Q8.8 multiply unit (combinational)
 *  - Q8.8 divide unit   (17 clock cycles)
 *  - Q8.8 sqrt unit     (13 clock cycles)
 *  - Kalman gain shortcut (A / (A+C), single command)
 *
 * ── Fixed-Point Format ────────────────────────────────────────────────────
 * Q8.8 signed (16-bit two's complement) throughout.
 * To encode a float f: write (int16_t)(f * 256.0)
 * To decode:           read  (float)raw / 256.0
 *
 * ── SPI Protocol ──────────────────────────────────────────────────────────
 * Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit framing.
 *
 *  ┌──────────────┬──────┬────────────────────────────────────────────────┐
 *  │ Command      │ Hex  │ Description                                    │
 *  ├──────────────┼──────┼────────────────────────────────────────────────┤
 *  │ LOAD_A       │ 0x10 │ Load operand A (2 bytes follow: MSB, LSB)      │
 *  │ LOAD_B       │ 0x11 │ Load operand B (2 bytes follow)                │
 *  │ LOAD_C       │ 0x12 │ Load operand C (2 bytes follow)                │
 *  │ LOAD_WEIGHT  │ 0x13 │ Load weight[idx]: 1 byte idx, 2 bytes data     │
 *  │ LOAD_INPUT   │ 0x14 │ Stream 8 input bytes → starts MAC when done    │
 *  ├──────────────┼──────┼────────────────────────────────────────────────┤
 *  │ OP_MUL       │ 0x20 │ result = A * B          (combinational)        │
 *  │ OP_DIV       │ 0x21 │ result = A / B          (17 clk)               │
 *  │ OP_MACDIV    │ 0x22 │ result = A / (A + C)    (Kalman K, 17 clk)     │
 *  │ OP_SQRT      │ 0x23 │ result = sqrt(A)        (13 clk, unsigned A)   │
 *  │ OP_DOT       │ 0x24 │ result = dot(weights, inputs)  (uses MAC unit) │
 *  ├──────────────┼──────┼────────────────────────────────────────────────┤
 *  │ READ_RESULT  │ 0x30 │ Read 2 bytes result (MSB then LSB)             │
 *  │ READ_STATUS  │ 0x31 │ Read 1 byte: [ovf|dbz|busy|done|mac_done|0x0]  │
 *  └──────────────┴──────┴────────────────────────────────────────────────┘
 *
 *  LOAD_WEIGHT byte sequence:  0x13  IDX(0-7)  DATA_HI  DATA_LO
 *  LOAD_INPUT  byte sequence:  0x14  IN0 IN1 IN2 IN3 IN4 IN5 IN6 IN7
 *    (8 unsigned Q0.8 input bytes; MAC starts automatically after IN7)
 *
 *  READ_RESULT:  First 0x30 returns HI byte on MISO for the NEXT SPI byte,
 *                then a dummy 0x00 returns the LO byte.
 *                i.e. the RP2040 does:  spi_transfer(0x30) → ignore
 *                                       spi_transfer(0x00) → hi
 *                                       spi_transfer(0x00) → lo
 *
 * ── Pin Mapping ───────────────────────────────────────────────────────────
 * uio[0]  SCK   in    SPI clock
 * uio[1]  MOSI  in    SPI data in
 * uio[2]  CS#   in    SPI chip select (active low)
 * uio[3]  MISO  out   SPI data out
 * uio[4]  BUSY  out   any operation in progress
 * uio[5]  DONE  out   result ready (1 clk pulse)
 * uio[6]  OVF   out   overflow (sticky, cleared on new op)
 * uio[7]  DBZ   out   divide by zero
 *
 * uo_out[7:0]  result[15:8] — high byte of result (for fast polling)
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ml_coprocessor (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    inout  wire [7:0] uio,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ================================================================
    // Pin wiring
    // ================================================================
    wire spi_sck  = uio[0];
    wire spi_mosi = uio[1];
    wire spi_cs_n = uio[2];
    wire spi_miso_w;

    wire busy_out, done_out, ovf_out, dbz_out;

    assign uio[3] = spi_miso_w;
    assign uio[4] = busy_out;
    assign uio[5] = done_out;
    assign uio[6] = ovf_out;
    assign uio[7] = dbz_out;

    // ================================================================
    // SPI Slave
    // ================================================================
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [7:0] tx_byte;

    spi_slave u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .spi_clk  (spi_sck),
        .spi_mosi (spi_mosi),
        .spi_cs_n (spi_cs_n),
        .spi_miso (spi_miso_w),
        .rx_byte  (rx_byte),
        .rx_valid (rx_valid),
        .tx_byte  (tx_byte)
    );

    // ================================================================
    // Operand registers
    // ================================================================
    reg signed [15:0] reg_a;
    reg signed [15:0] reg_b;
    reg signed [15:0] reg_c;
    reg signed [15:0] result;   // last computed result

    // ================================================================
    // Multiplier (combinational)
    // ================================================================
    reg  signed [15:0] mul_a, mul_b;
    wire signed [15:0] mul_result;
    wire               mul_ovf;

    fixedpoint_mul u_mul (
        .a(mul_a), .b(mul_b), .result(mul_result), .ovf(mul_ovf)
    );

    // ================================================================
    // Divider (17 clk cycles)
    // ================================================================
    reg  signed [15:0] div_num, div_den;
    reg                div_start;
    wire signed [15:0] div_result;
    wire               div_done, div_dbz, div_ovf;

    fixedpoint_div u_div (
        .clk(clk), .rst_n(rst_n),
        .start(div_start),
        .dividend(div_num), .divisor(div_den),
        .result(div_result),
        .done(div_done), .div_by_zero(div_dbz), .ovf(div_ovf)
    );

    // ================================================================
    // Square Root (13 clk cycles)
    // ================================================================
    reg  [15:0] sqrt_in;
    reg         sqrt_start;
    wire [15:0] sqrt_result;
    wire        sqrt_done;

    fixedpoint_sqrt u_sqrt (
        .clk(clk), .rst_n(rst_n),
        .start(sqrt_start),
        .radicand(sqrt_in),
        .result(sqrt_result),
        .done(sqrt_done)
    );

    // ================================================================
    // MAC Array
    // ================================================================
    reg         mac_weight_load;
    reg [2:0]   mac_weight_idx;
    reg signed [15:0] mac_weight_data;
    reg         mac_start;
    reg [7:0]   mac_input_byte;
    reg         mac_input_valid;
    wire signed [15:0] mac_result;
    wire        mac_done, mac_busy;

    mac_array u_mac (
        .clk(clk), .rst_n(rst_n),
        .weight_load(mac_weight_load),
        .weight_idx(mac_weight_idx),
        .weight_data(mac_weight_data),
        .start(mac_start),
        .input_byte(mac_input_byte),
        .input_valid(mac_input_valid),
        .acc_result(mac_result),
        .done(mac_done),
        .busy(mac_busy)
    );

    // ================================================================
    // Command Decoder
    // ================================================================
    // Command codes
    localparam CMD_LOAD_A    = 8'h10;
    localparam CMD_LOAD_B    = 8'h11;
    localparam CMD_LOAD_C    = 8'h12;
    localparam CMD_LOAD_W    = 8'h13;
    localparam CMD_LOAD_IN   = 8'h14;
    localparam CMD_MUL       = 8'h20;
    localparam CMD_DIV       = 8'h21;
    localparam CMD_MACDIV    = 8'h22;
    localparam CMD_SQRT      = 8'h23;
    localparam CMD_DOT       = 8'h24;
    localparam CMD_READ_RES  = 8'h30;
    localparam CMD_READ_STAT = 8'h31;

    // Parser states
    localparam P_CMD         = 4'd0;
    localparam P_LOAD_A_HI   = 4'd1;
    localparam P_LOAD_A_LO   = 4'd2;
    localparam P_LOAD_B_HI   = 4'd3;
    localparam P_LOAD_B_LO   = 4'd4;
    localparam P_LOAD_C_HI   = 4'd5;
    localparam P_LOAD_C_LO   = 4'd6;
    localparam P_LOAD_W_IDX  = 4'd7;
    localparam P_LOAD_W_HI   = 4'd8;
    localparam P_LOAD_W_LO   = 4'd9;
    localparam P_LOAD_IN     = 4'd10;
    localparam P_READ_LO     = 4'd11;

    reg [3:0]  parse_state;
    reg [7:0]  tmp_hi;          // stash MSB of 16-bit loads
    reg [2:0]  in_cnt;          // input byte counter for MAC streaming
    reg        busy_r;
    reg        done_r;
    reg        ovf_r;
    reg        dbz_r;
    reg        mac_done_latch;
    reg        waiting_div;
    reg        waiting_sqrt;

    // ── Main state machine ───────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parse_state     <= P_CMD;
            reg_a           <= 16'h0000;
            reg_b           <= 16'h0100;  // 1.0
            reg_c           <= 16'h0000;
            result          <= 16'h0000;
            busy_r          <= 1'b0;
            done_r          <= 1'b0;
            ovf_r           <= 1'b0;
            dbz_r           <= 1'b0;
            mac_done_latch  <= 1'b0;
            waiting_div     <= 1'b0;
            waiting_sqrt    <= 1'b0;
            div_start       <= 1'b0;
            sqrt_start      <= 1'b0;
            mac_weight_load <= 1'b0;
            mac_start       <= 1'b0;
            mac_input_valid <= 1'b0;
            tx_byte         <= 8'h00;
            in_cnt          <= 3'd0;
            tmp_hi          <= 8'h00;
        end else begin
            // Default strobes to 0
            div_start       <= 1'b0;
            sqrt_start      <= 1'b0;
            mac_weight_load <= 1'b0;
            mac_start       <= 1'b0;
            mac_input_valid <= 1'b0;
            done_r          <= 1'b0;

            // ── Capture async unit completions ───────────────────────
            if (div_done && waiting_div) begin
                result      <= div_result;
                ovf_r       <= ovf_r | div_ovf;
                dbz_r       <= div_dbz;
                busy_r      <= 1'b0;
                done_r      <= 1'b1;
                waiting_div <= 1'b0;
            end

            if (sqrt_done && waiting_sqrt) begin
                result       <= sqrt_result;
                busy_r       <= 1'b0;
                done_r       <= 1'b1;
                waiting_sqrt <= 1'b0;
            end

            if (mac_done) begin
                result         <= mac_result;
                busy_r         <= 1'b0;
                done_r         <= 1'b1;
                mac_done_latch <= 1'b1;
            end

            // ── SPI byte received ────────────────────────────────────
            if (rx_valid) begin
                case (parse_state)

                // ── Command byte ─────────────────────────────────────
                P_CMD: begin
                    ovf_r  <= 1'b0;
                    dbz_r  <= 1'b0;

                    case (rx_byte)
                        CMD_LOAD_A:    parse_state <= P_LOAD_A_HI;
                        CMD_LOAD_B:    parse_state <= P_LOAD_B_HI;
                        CMD_LOAD_C:    parse_state <= P_LOAD_C_HI;
                        CMD_LOAD_W:    parse_state <= P_LOAD_W_IDX;

                        CMD_LOAD_IN: begin
                            // Start MAC accumulation; stream 8 bytes
                            mac_start   <= 1'b1;
                            in_cnt      <= 3'd0;
                            busy_r      <= 1'b1;
                            parse_state <= P_LOAD_IN;
                        end

                        CMD_MUL: begin
                            // Combinational; result registered next cycle
                            mul_a <= reg_a;
                            mul_b <= reg_b;
                            // We capture mul_result one cycle later via
                            // the "pending_mul" path below.
                            // For simplicity: result is just registered here.
                            result <= mul_result;  // combinational read
                            ovf_r  <= mul_ovf;
                            done_r <= 1'b1;
                        end

                        CMD_DIV: begin
                            div_num     <= reg_a;
                            div_den     <= reg_b;
                            div_start   <= 1'b1;
                            busy_r      <= 1'b1;
                            waiting_div <= 1'b1;
                        end

                        CMD_MACDIV: begin
                            // Kalman gain: K = P / (P + R)
                            // reg_a = P, reg_c = R
                            div_num     <= reg_a;
                            div_den     <= reg_a + reg_c;
                            div_start   <= 1'b1;
                            busy_r      <= 1'b1;
                            waiting_div <= 1'b1;
                        end

                        CMD_SQRT: begin
                            sqrt_in      <= reg_a;  // treat as unsigned
                            sqrt_start   <= 1'b1;
                            busy_r       <= 1'b1;
                            waiting_sqrt <= 1'b1;
                        end

                        CMD_DOT: begin
                            // Alias for LOAD_IN sequence if inputs already loaded
                            // Alternatively: trigger MAC with existing input buffer
                            // (Simplification: RP2040 uses CMD_LOAD_IN which auto-starts)
                            done_r <= mac_done_latch;
                            result <= mac_result;
                        end

                        CMD_READ_RES: begin
                            // Pre-load tx_byte with high byte; RP2040 sends
                            // one more dummy byte to clock out the low byte.
                            tx_byte     <= result[15:8];
                            parse_state <= P_READ_LO;
                        end

                        CMD_READ_STAT: begin
                            tx_byte <= {ovf_r, dbz_r, busy_r, done_r,
                                        mac_done_latch, 3'b000};
                        end

                        default:;  // unknown command, stay in CMD state
                    endcase
                end

                // ── Operand A loading ────────────────────────────────
                P_LOAD_A_HI: begin tmp_hi <= rx_byte; parse_state <= P_LOAD_A_LO; end
                P_LOAD_A_LO: begin
                    reg_a       <= {tmp_hi, rx_byte};
                    // Also update combinational multiplier inputs in case
                    // next command is MUL
                    mul_a       <= {tmp_hi, rx_byte};
                    parse_state <= P_CMD;
                end

                // ── Operand B loading ────────────────────────────────
                P_LOAD_B_HI: begin tmp_hi <= rx_byte; parse_state <= P_LOAD_B_LO; end
                P_LOAD_B_LO: begin
                    reg_b       <= {tmp_hi, rx_byte};
                    mul_b       <= {tmp_hi, rx_byte};
                    parse_state <= P_CMD;
                end

                // ── Operand C loading ────────────────────────────────
                P_LOAD_C_HI: begin tmp_hi <= rx_byte; parse_state <= P_LOAD_C_LO; end
                P_LOAD_C_LO: begin reg_c <= {tmp_hi, rx_byte}; parse_state <= P_CMD; end

                // ── Weight loading: IDX → HI → LO ───────────────────
                P_LOAD_W_IDX: begin
                    mac_weight_idx <= rx_byte[2:0];  // only 3 bits (0..7)
                    parse_state    <= P_LOAD_W_HI;
                end
                P_LOAD_W_HI: begin tmp_hi <= rx_byte; parse_state <= P_LOAD_W_LO; end
                P_LOAD_W_LO: begin
                    mac_weight_data <= {tmp_hi, rx_byte};
                    mac_weight_load <= 1'b1;
                    parse_state     <= P_CMD;
                end

                // ── Input streaming for MAC ──────────────────────────
                P_LOAD_IN: begin
                    mac_input_byte  <= rx_byte;
                    mac_input_valid <= 1'b1;
                    if (in_cnt == 3'd7) begin
                        parse_state <= P_CMD;
                        // MAC unit will assert mac_done a few cycles later
                    end else begin
                        in_cnt <= in_cnt + 1'b1;
                    end
                end

                // ── Result low byte ──────────────────────────────────
                P_READ_LO: begin
                    tx_byte     <= result[7:0];
                    parse_state <= P_CMD;
                end

                default: parse_state <= P_CMD;
                endcase
            end
        end
    end

    // ================================================================
    // Output assignments
    // ================================================================
    assign uo_out   = result[15:8];
    assign busy_out = busy_r | mac_busy;
    assign done_out = done_r;
    assign ovf_out  = ovf_r;
    assign dbz_out  = dbz_r;

endmodule
