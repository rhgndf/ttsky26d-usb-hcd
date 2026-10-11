/*
 * spi_slave.v - SPI mode-0 register-port slave (MSB first, SCK <= clk/12)
 * Byte 0: {RW, A[6:0]}; then data bytes; address auto-increments except at 0xA.
 * Read bytes are prefetched from the register file after the address has
 * settled; the read side-effect strobe (re) fires at the byte's 8th SCK edge.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sck,
    input  wire       copi,
    input  wire       cs_n,
    output wire       cipo,
    output wire       active,
    output reg  [3:0] addr,
    output wire [7:0] wdata,
    output reg        we,
    output reg        re,
    input  wire [7:0] rdata
);

    reg [1:0] sck_s, copi_s, cs_s;
    always @(posedge clk) begin
        if (!rst_n) begin
            sck_s <= 0;
            copi_s <= 0;
            cs_s <= 2'b11;
        end else begin
            sck_s  <= {sck_s[0], sck};
            copi_s <= {copi_s[0], copi};
            cs_s   <= {cs_s[0], cs_n};
        end
    end
    wire sck_rise = (sck_s == 2'b01);
    wire sck_fall = (sck_s == 2'b10);
    wire cs_low   = ~cs_s[1];
    assign active = cs_low;

    reg [7:0] sr_in;      // COPI shift register
    reg [7:0] sr_out;     // CIPO shift register
    reg [2:0] bit_cnt;
    reg       first;      // first byte after CS fall = command
    reg       rd_op;      // command was a read
    reg       load_next;  // load rdata into sr_out at the byte boundary
    reg       advance_pending;

    reg [7:0] wdata_reg;
    assign cipo  = cs_low ? sr_out[7] : 1'b0;
    assign wdata = wdata_reg; // byte captured at the 8th rising edge

    wire [7:0] in_byte = {sr_in[6:0], copi_s[1]};

    always @(posedge clk) begin
        if (!rst_n) begin
            sr_in    <= 0;
            sr_out   <= 0;
            bit_cnt  <= 0;
            first    <= 1;
            rd_op    <= 0;
            addr     <= 0;
            we       <= 0;
            re       <= 0;
            load_next <= 0;
            advance_pending <= 0;
            wdata_reg <= 0;
        end else begin
            we <= 0;
            re <= 0;
            // Hold the current address until the register file consumes its strobe.
            if (advance_pending) begin
                if (addr != 4'hA)
                    addr <= addr + 1;
                advance_pending <= 0;
            end
            if (!cs_low) begin
                bit_cnt  <= 0;
                first    <= 1;
                sr_out   <= 0;
                load_next <= 0;
                advance_pending <= 0;
            end else begin
                if (sck_rise) begin
                    sr_in   <= in_byte;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 7) begin
                        // 8th rising edge: byte complete
                        if (first) begin
                            first <= 0;
                            rd_op <= in_byte[7];
                            addr  <= in_byte[3:0];
                            if (in_byte[7])
                                load_next <= 1; // first read byte
                        end else begin
                            wdata_reg <= in_byte;
                            if (rd_op)
                                re <= 1;
                            else
                                we <= 1;
                            advance_pending <= 1;
                            if (rd_op)
                                load_next <= 1;
                        end
                    end
                end
                if (sck_fall) begin
                    // At a byte boundary load the prefetched byte; the side
                    // effect (re) and address advance have already settled.
                    if (load_next && bit_cnt == 0) begin
                        sr_out    <= rdata;
                        load_next <= 0;
                    end else if (bit_cnt != 0)
                        sr_out <= {sr_out[6:0], 1'b0};
                end
            end
        end
    end

    wire _unused = &{copi_s, 1'b0};

endmodule
