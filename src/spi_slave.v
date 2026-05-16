/*
 * spi_slave.v — SPI Mode 0, MSB-first byte receiver/transmitter
 *
 * Presents a simple 8-bit shift register interface.
 * rx_valid pulses for one clk cycle when a full byte is received.
 * tx_data should be loaded before CS goes low (or updated on rx_valid).
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

module spi_slave (
    input  wire       clk,        // system clock
    input  wire       rst_n,
    // SPI bus
    input  wire       spi_clk,    // SCK from RP2040
    input  wire       spi_mosi,   // MOSI
    input  wire       spi_cs_n,   // CS# (active low)
    output reg        spi_miso,   // MISO
    // internal interface
    output reg  [7:0] rx_byte,    // last received byte
    output reg        rx_valid,   // pulses 1 clk when byte complete
    input  wire [7:0] tx_byte     // byte to send on next transfer
);

    // ----------------------------------------------------------------
    // Edge detection on SCK (sample on rising, shift out on falling)
    // We re-synchronise SCK into our clock domain with a 2-FF synchroniser.
    // ----------------------------------------------------------------
    reg [2:0] sck_sync;
    reg [1:0] cs_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync <= 3'b000;
            cs_sync  <= 2'b11;
        end else begin
            sck_sync <= {sck_sync[1:0], spi_clk};
            cs_sync  <= {cs_sync[0],    spi_cs_n};
        end
    end

    wire sck_rising  = (sck_sync[2:1] == 2'b01);
    wire sck_falling = (sck_sync[2:1] == 2'b10);
    wire cs_active   = ~cs_sync[1];

    // ----------------------------------------------------------------
    // Shift register
    // ----------------------------------------------------------------
    reg [7:0] rx_shift;
    reg [7:0] tx_shift;
    reg [2:0] bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_shift  <= 8'h00;
            tx_shift  <= 8'h00;
            bit_cnt   <= 3'd0;
            rx_byte   <= 8'h00;
            rx_valid  <= 1'b0;
            spi_miso  <= 1'b0;
        end else begin
            rx_valid <= 1'b0;   // default: not valid

            if (!cs_active) begin
                // CS deasserted: preload tx shift register and reset counter
                tx_shift <= tx_byte;
                bit_cnt  <= 3'd0;
                spi_miso <= tx_byte[7];   // drive MSB ready for first SCK
            end else begin
                if (sck_rising) begin
                    // Sample MOSI on rising edge
                    rx_shift <= {rx_shift[6:0], spi_mosi};

                    if (bit_cnt == 3'd7) begin
                        // Full byte received
                        rx_byte  <= {rx_shift[6:0], spi_mosi};
                        rx_valid <= 1'b1;
                        bit_cnt  <= 3'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                if (sck_falling) begin
                    // Shift out MISO on falling edge
                    tx_shift <= {tx_shift[6:0], 1'b0};
                    spi_miso <= tx_shift[6];   // next bit
                end
            end
        end
    end

endmodule
