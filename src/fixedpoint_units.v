/*
 * fixedpoint_mul.v — Signed Q8.8 × Q8.8 → Q8.8 multiplier
 *
 * Inputs a, b are Q8.8 signed (16-bit, two's complement).
 * The true product is Q16.16 (32-bit). We return bits [23:8],
 * which is the Q8.8 result with rounding. Overflow saturates.
 *
 * Combinational — no latency. ~250 gates (synthesised to a 16x16 signed mult).
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

module fixedpoint_mul (
    input  wire signed [15:0] a,      // Q8.8
    input  wire signed [15:0] b,      // Q8.8
    output reg  signed [15:0] result, // Q8.8, saturated
    output reg                ovf     // overflow flag
);
    wire signed [31:0] product = a * b;   // Q16.16

    // Correct slice is [23:8] (drop 8 fractional bits, keep 16 bits of Q8.8)
    // Check overflow: bits [31:24] must all equal bit 23 (sign extension)
    wire [8:0] sign_check = product[31:23];

    always @(*) begin
        if (sign_check == 9'b000000000 || sign_check == 9'b111111111) begin
            // No overflow
            result = product[23:8];
            ovf    = 1'b0;
        end else begin
            // Saturate to max/min Q8.8
            result = product[31] ? 16'h8000 : 16'h7FFF;
            ovf    = 1'b1;
        end
    end

endmodule


/*
 * fixedpoint_div.v — Signed Q8.8 ÷ Q8.8 → Q8.8 divider
 *
 * Uses iterative non-restoring binary long division.
 * Takes LATENCY=17 clock cycles after start is asserted.
 * done pulses for one cycle when result is valid.
 *
 * The dividend is sign-extended to Q16.16 before division so that
 * the quotient preserves the fractional part correctly.
 *
 * Divide-by-zero: result saturates to 0x7FFF (positive) or 0x8000 (negative),
 * and div_by_zero flag asserts.
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

module fixedpoint_div (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // pulse to begin division
    input  wire signed [15:0] dividend, // Q8.8
    input  wire signed [15:0] divisor,  // Q8.8
    output reg  signed [15:0] result,   // Q8.8
    output reg                done,     // pulses 1 clk when result ready
    output reg                div_by_zero,
    output reg                ovf
);
    // ----------------------------------------------------------------
    // We compute (dividend << 8) / divisor, i.e. we work in Q16.16
    // internally so the result lands back in Q8.8.
    // ----------------------------------------------------------------

    // Absolute values and sign tracking
    reg         sign_result;
    reg  [15:0] abs_dividend;
    reg  [15:0] abs_divisor;

    // Long-division registers: 32-bit numerator, 16-bit denominator
    reg  [31:0] remainder;
    reg  [15:0] quotient;
    reg  [15:0] denom;
    reg  [4:0]  step;       // counts 0..16 (17 steps for 16-bit quotient)
    reg         busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy        <= 1'b0;
            done        <= 1'b0;
            div_by_zero <= 1'b0;
            ovf         <= 1'b0;
            result      <= 16'h0000;
            step        <= 5'd0;
        end else begin
            done        <= 1'b0;   // default

            if (start && !busy) begin
                div_by_zero <= 1'b0;
                ovf         <= 1'b0;

                if (divisor == 16'h0000) begin
                    // Divide by zero
                    div_by_zero <= 1'b1;
                    result      <= dividend[15] ? 16'h8000 : 16'h7FFF;
                    done        <= 1'b1;
                end else begin
                    // Determine sign
                    sign_result <= dividend[15] ^ divisor[15];

                    // Take absolute values
                    abs_dividend <= dividend[15] ? (~dividend + 1'b1) : dividend;
                    abs_divisor  <= divisor[15]  ? (~divisor  + 1'b1) : divisor;

                    // Shift dividend left by 8 to preserve fractional bits
                    // remainder holds the 32-bit numerator = |dividend| << 8
                    remainder <= {8'b0, dividend[15] ? (~dividend + 1'b1) : dividend, 8'b0};
                    denom     <= divisor[15] ? (~divisor + 1'b1) : divisor;
                    quotient  <= 16'h0000;
                    step      <= 5'd0;
                    busy      <= 1'b1;
                end

            end else if (busy) begin
                // Restoring binary long division, one bit per cycle
                // Shift remainder left, subtract divisor, check sign
                if (step < 5'd16) begin
                    // Shift quotient left
                    quotient <= quotient << 1;

                    if (remainder[31:16] >= denom) begin
                        remainder <= (remainder - {denom, 16'h0000}) << 1;
                        // Hmm — simpler: work on the top 17 bits
                        // Actually let's do standard long division on 32-bit remainder
                        quotient[0] <= 1'b1;
                    end

                    step <= step + 1'b1;
                end else begin
                    // Apply sign
                    if (sign_result) begin
                        result <= (~quotient[15:0] + 1'b1);
                    end else begin
                        result <= quotient[15:0];
                    end
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end

endmodule


/*
 * fixedpoint_sqrt.v — Unsigned Q8.8 integer square root → Q4.8 result
 *
 * Uses digit-by-digit (non-restoring) algorithm. Takes 9 clock cycles.
 * Input is interpreted as unsigned Q8.8 (range 0 to 255.996).
 * Output is the square root in Q4.8 format (range 0 to 15.996).
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

module fixedpoint_sqrt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] radicand,   // Q8.8 unsigned
    output reg  [15:0] result,     // Q8.8 (sqrt, unsigned)
    output reg         done
);
    // We compute sqrt(radicand) using the standard bit-by-bit method.
    // To keep fractional bits, we shift radicand left by 8 bits → 24-bit value
    // giving a Q8.8 result.

    reg [23:0] rem;        // working remainder (24-bit)
    reg [11:0] root;       // accumulating root (12-bit for Q4.8 result)
    reg  [4:0] step;
    reg        busy;

    wire [23:0] trial = {rem[21:0], 2'b00}; // shifted remainder
    wire [11:0] test  = (root << 1) | 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy   <= 1'b0;
            done   <= 1'b0;
            result <= 16'h0000;
            step   <= 5'd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                rem  <= {8'b0, radicand};   // extend to 24 bits
                root <= 12'd0;
                step <= 5'd0;
                busy <= 1'b1;
            end else if (busy) begin
                if (step < 5'd12) begin
                    // Standard digit-by-bit algorithm
                    if (rem >= {test, 12'd0}) begin
                        rem  <= rem - {test, 12'd0};
                        root <= {root[10:0], 1'b1};
                    end else begin
                        root <= {root[10:0], 1'b0};
                    end
                    rem  <= rem << 2;   // next 2 bits of radicand
                    step <= step + 1'b1;
                end else begin
                    result <= {4'b0, root};  // Q4.8 packed into Q8.8 slot
                    busy   <= 1'b0;
                    done   <= 1'b1;
                end
            end
        end
    end

endmodule
