/*
 * mac_array.v — 8-element signed Q8.8 multiply-accumulate array
 *
 * Computes dot product:  acc = Σ (weight[i] * input[i])  for i=0..7
 *
 * Weights are loaded via SPI (externally) and held in registers.
 * Each clock cycle, one 8-bit input byte is clocked in (start with i=0).
 * After 8 cycles, done pulses and acc holds the Q8.8 result.
 *
 * Inputs are unsigned Q0.8 (0 to 0.996), weights are signed Q8.8.
 * The partial products are Q8.16; we accumulate in Q8.16 (32-bit)
 * then truncate to Q8.8 with saturation.
 *
 * Interface:
 *   weight_load   — load weight[weight_idx] = weight_data
 *   start         — begin accumulation (resets accumulator)
 *   input_byte    — one input value per cycle (latched on rising clk when busy)
 *   input_valid   — qualifier for input_byte
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

module mac_array (
    input  wire        clk,
    input  wire        rst_n,

    // Weight loading interface
    input  wire        weight_load,        // load strobe
    input  wire [2:0]  weight_idx,         // which weight (0..7)
    input  wire signed [15:0] weight_data, // Q8.8 value

    // Accumulation interface
    input  wire        start,              // reset acc and begin
    input  wire [7:0]  input_byte,         // Q0.8 unsigned input
    input  wire        input_valid,        // strobe per input byte

    // Result
    output reg  signed [15:0] acc_result,  // Q8.8 dot product
    output reg                done,        // pulses when all 8 inputs consumed
    output reg                busy
);

    // ── Weight register file: 8 × 16-bit signed ─────────────────────
    reg signed [15:0] weights [0:7];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 8; k = k + 1)
                weights[k] <= 16'h0100;  // default weight = 1.0
        end else if (weight_load) begin
            weights[weight_idx] <= weight_data;
        end
    end

    // ── Accumulator ─────────────────────────────────────────────────
    reg signed [31:0] acc;      // Q8.16 accumulator (wider to avoid overflow)
    reg [2:0]  input_cnt;       // counts 0..7

    // Partial product: signed weight × unsigned input → signed Q8.16
    // Multiply: 16-bit signed × 8-bit unsigned = 24-bit signed
    // Sign-extend to 32-bit for accumulation
    wire signed [15:0] cur_weight = weights[input_cnt];
    wire signed [23:0] partial    = cur_weight * $signed({1'b0, input_byte}); // 16s × 9u

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc       <= 32'h00000000;
            input_cnt <= 3'd0;
            done      <= 1'b0;
            busy      <= 1'b0;
            acc_result <= 16'h0000;
        end else begin
            done <= 1'b0;

            if (start) begin
                acc       <= 32'h00000000;
                input_cnt <= 3'd0;
                busy      <= 1'b1;
            end else if (busy && input_valid) begin
                // Accumulate: partial is Q8.8 (weight) × Q0.8 (input) = Q8.16
                // sign-extend partial to 32 bits
                acc       <= acc + {{8{partial[23]}}, partial};
                input_cnt <= input_cnt + 1'b1;

                if (input_cnt == 3'd7) begin
                    // All 8 inputs consumed: truncate Q8.16 → Q8.8
                    // Result is in acc[23:8]
                    if (acc[31:24] == 8'hFF || acc[31:24] == 8'h00) begin
                        acc_result <= acc[23:8];
                    end else begin
                        // Saturation
                        acc_result <= acc[31] ? 16'h8000 : 16'h7FFF;
                    end
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end

endmodule
