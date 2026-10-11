/*
 * spi_slave.v - SPI mode-0 register-port slave (MSB first, SCK <= clk/12)
 * Byte 0: {RW, A[6:0]}; then data bytes; address auto-increments except at 0xA.
 * Single shared shift register: the bit the master sampled at the rising
 * edge is shifted in on the falling edge, so sr[7] (CIPO) only changes in
 * the SCK low half. Read bytes are prefetched from the register file after
 * the address has settled; the read side-effect strobe (re) fires at the
 * byte's 8th rising edge.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       copi,
    input  wire       sck,
    input  wire       cs_n,
    output wire       cipo,
    output wire       active,
    output reg  [3:0] addr,
    output wire [7:0] wdata,
    output wire       we,
    output wire       re,
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

    reg [7:0] sr;         // shared shift register: COPI in on falls, sr[7] is CIPO
    reg [2:0] bit_cnt;
    reg       first;      // first byte after CS fall = command
    reg       rd_op;      // command was a read
    reg       load_next;  // load rdata into sr at the next falling edge

    assign cipo  = cs_low ? sr[7] : 1'b0;
    // COPI byte completes at the 8th rising edge and stays valid through the
    // SCK high phase, which is when we/re commit.
    assign wdata = {sr[6:0], copi_s[1]};
    // Combinational strobes: coincident with the still-current addr
    wire byte_done = cs_low && sck_rise && (bit_cnt == 7) && !first;
    assign we = byte_done && !rd_op;
    assign re = byte_done &&  rd_op;

    always @(posedge clk) begin
        if (!rst_n) begin
            sr       <= 0;
            bit_cnt  <= 0;
            first    <= 1;
            rd_op    <= 0;
            addr     <= 0;
            load_next <= 0;
        end else begin
            if (!cs_low) begin
                bit_cnt <= 0;
                first   <= 1;
                sr      <= 0;
                load_next <= 0;
            end else if (sck_rise) begin
                if (bit_cnt == 7) begin
                    // 8th rising edge: byte complete (wdata valid now)
                    if (first) begin
                        first <= 0;
                        rd_op <= wdata[7];
                        addr  <= wdata[3:0];
                        if (wdata[7])
                            load_next <= 1; // first read byte
                    end else begin
                        if (addr != 4'hA)
                            addr <= addr + 1;
                        if (rd_op)
                            load_next <= 1;
                    end
                end
            end else if (sck_fall) begin
                bit_cnt <= bit_cnt + 1;
                if (load_next) begin
                    // Prefetched read byte; addr has already advanced.
                    sr <= rdata;
                    load_next <= 0;
                end else
                    sr <= {sr[6:0], copi_s[1]};
            end
        end
    end

    wire _unused = &{1'b0};

endmodule
