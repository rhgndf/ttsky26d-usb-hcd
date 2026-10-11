/*
 * i2c_slave.v - I2C register-port slave, 7-bit addr {5'b10110, A1, A0}
 * Register-pointer protocol: write [reg] [data...], or write [reg] then
 * repeated-START + read. Address auto-increments except at 0xA.
 * `re` fires at the 9th (ACK) clock of each transmitted read byte.
 * No clock stretching. Up to 1 MHz SCL.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module i2c_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       scl,
    input  wire       sda_in,
    input  wire [1:0] i2c_addr,   // A1:A0
    output reg        sda_oe,     // open-drain: 1 = drive SDA low
    output reg  [3:0] addr,
    output wire [7:0] wdata,
    output wire       we,
    output wire       re,
    input  wire [7:0] rdata
);

    // 2-FF sync + 3-sample majority glitch filter
    reg [1:0] scl_s, sda_s;
    reg scl_f, sda_f, scl_p, sda_pp;
    always @(posedge clk) begin
        if (!rst_n) begin
            scl_s <= 2'b11;
            sda_s <= 2'b11;
            scl_f <= 1;
            sda_f <= 1;
            scl_p <= 1;
            sda_pp <= 1;
        end else begin
            scl_s <= {scl_s[0], scl};
            sda_s <= {sda_s[0], sda_in};
            if ((scl_s[1] && scl_s[0]) || (scl_s[1] && scl_f) || (scl_s[0] && scl_f))
                scl_f <= 1;
            else
                scl_f <= 0;
            if ((sda_s[1] && sda_s[0]) || (sda_s[1] && sda_f) || (sda_s[0] && sda_f))
                sda_f <= 1;
            else
                sda_f <= 0;
            scl_p  <= scl_f;
            sda_pp <= sda_f;
        end
    end
    wire scl_rise = scl_f && !scl_p;
    wire scl_fall = !scl_f && scl_p;
    wire sda_rise = sda_f && !sda_pp;
    wire sda_fall = !sda_f && sda_pp;
    wire start_ev = sda_fall && scl_f;
    wire stop_ev  = sda_rise && scl_f;

    localparam ST_IDLE = 0, ST_ADDR = 1, ST_ACKA = 2,
               ST_RX   = 3, ST_ACKD = 4, ST_TX = 5,
               ST_TXA  = 6, ST_WAIT = 7, ST_ACKB = 8, ST_ACKD2 = 9;
    reg [3:0] state;

    reg [7:0] sr;         // shared shift register (rx and tx)
    reg [2:0] bit_cnt;
    reg       rw;         // 1 = master reads
    reg       got_ptr;    // register pointer received
    reg       master_ack;

    // byte completes at the 8th SCL rising edge; stays valid while SCL is
    // high, which is when we commits
    assign wdata = {sr[6:0], sda_f};
    wire [7:0] in_byte = wdata;
    // Combinational strobes: coincident with the still-current addr
    assign we = (state == ST_RX) && scl_rise && (bit_cnt == 7) && got_ptr;
    assign re = (state == ST_TXA) && scl_rise;

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= ST_IDLE;
            sr      <= 0;
            bit_cnt <= 0;
            rw      <= 0;
            got_ptr <= 0;
            addr    <= 0;
            sda_oe  <= 0;
            master_ack <= 0;
        end else begin

            if (stop_ev) begin
                state   <= ST_IDLE;
                sda_oe  <= 0;
                got_ptr <= 0;
                bit_cnt <= 0;
            end else if (start_ev) begin
                state   <= ST_ADDR;
                bit_cnt <= 0;
                sda_oe  <= 0;
            end else begin
                case (state)
                    ST_ADDR: if (scl_rise) begin
                        sr <= in_byte;
                        bit_cnt <= bit_cnt + 1;
                        if (bit_cnt == 7) begin
                            bit_cnt <= 0;
                            if (in_byte[7:1] == {5'b10110, i2c_addr}) begin
                                state  <= ST_ACKA;
                                rw     <= in_byte[0];
                            end else begin
                                state <= ST_WAIT; // not us
                            end
                        end
                    end

                    // Hold SDA low through the whole 9th (ACK) clock: the
                    // first falling edge starts the ACK bit, the second ends it.
                    ST_ACKA: if (scl_fall) begin
                        sda_oe <= 1; // ACK, asserted only in the low half
                        if (rw) begin
                            state  <= ST_TX;
                            sr     <= rdata; // prefetch read byte
                        end else begin
                            state  <= ST_ACKB;
                        end
                    end
                    ST_ACKB: if (scl_fall) begin
                        sda_oe <= 0;
                        state  <= ST_RX;
                    end

                    ST_RX: if (scl_rise) begin
                        sr <= in_byte;
                        bit_cnt <= bit_cnt + 1;
                        if (bit_cnt == 7) begin
                            bit_cnt <= 0;
                            state   <= ST_ACKD;
                            if (!got_ptr) begin
                                got_ptr <= 1;
                                addr    <= in_byte[3:0];
                            end else begin
                                if (addr != 4'hA)
                                    addr <= addr + 1;
                            end
                        end
                    end

                    ST_ACKD: if (scl_fall) begin
                        sda_oe <= 1; // ACK, asserted only in the low half
                        state <= ST_ACKD2;
                    end
                    ST_ACKD2: if (scl_fall) begin
                        sda_oe <= 0;
                        state  <= ST_RX;
                    end

                    ST_TX: begin
                        // Advance to the next bit on SCL falling edges.
                        if (scl_fall) begin
                            sda_oe  <= ~sr[7];
                            sr      <= {sr[6:0], 1'b1};
                            bit_cnt <= bit_cnt + 1;
                            if (bit_cnt == 7) begin
                                bit_cnt <= 0;
                                state   <= ST_TXA;
                                sda_oe  <= 0; // release for master ACK
                            end
                        end
                    end

                    ST_TXA: begin
                        if (scl_rise) begin
                            // The ninth clock completes the byte even for a NAK.
                            master_ack <= !sda_f;
                            if (addr != 4'hA)
                                addr <= addr + 1;
                        end
                        if (scl_fall) begin
                            if (master_ack) begin
                                state <= ST_TX;
                                bit_cnt <= 0;
                                sr <= rdata; // prefetch next byte (addr settled)
                            end else begin
                                state <= ST_WAIT;
                            end
                        end
                    end

                    ST_WAIT: sda_oe <= 0;
                    default: state <= ST_IDLE;
                endcase
            end
        end
    end

    // re strobe fires at the 9th clock: implemented above at bit_cnt==7->TXA
    // (one bit early visually but exactly once per transferred byte)

    wire _unused = &{bit_cnt, 1'b0};

endmodule
