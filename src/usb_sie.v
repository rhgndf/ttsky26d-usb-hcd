/*
 * usb_sie.v - streaming USB full-speed PHY and host/device SIE.
 * No packet storage: RX bytes stream straight into the register buffer,
 * TX bytes stream out of it. One shared shift/crc/ones datapath for
 * half-duplex RX and TX. All line I/O synchronous to the 48 MHz clk.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module usb_sie (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        dp_in,
    input  wire        dm_in,
    input  wire [5:0]  ctrl,
    input  wire [6:0]  addr_reg,
    input  wire [3:0]  cmd_ep,
    input  wire [1:0]  cmd_op,
    input  wire        cmd_data1,
    input  wire        cmd_any,
    input  wire        cmd_wr,
    input  wire [3:0]  len_reg,
    input  wire [7:0]  buf_rdata,
    output reg         busy,
    output reg         dp_sync,
    output reg         dm_sync,
    output reg         se0_long,
    output reg  [10:0] frame,
    output reg         buf_we,
    output reg  [7:0]  buf_wdata,
    output reg         buf_inc,
    output reg         bufptr_clr,
    output reg         op_done,
    output reg         res_wr,
    output reg  [7:0]  res_val,
    output reg         tok_wr,
    output wire [7:0]  tok_val,
    output reg         len_wr,
    output wire [3:0]  len_val,
    output reg  [3:0]  irq_set,
    output wire        tx_dp,
    output wire        tx_dm,
    output wire        tx_oe,
    output wire        tx_active
);

    // TX engine states
    localparam TX_IDLE=2'd0, TX_BITS=2'd1, TX_EOP0=2'd2, TX_EOPJ=2'd3;
    // Transaction FSM states
    localparam S_IDLE=3'd0, S_WAIT=3'd1, S_DEVACK=3'd2, S_DEVDATA=3'd3;
    // TX job kinds
    localparam TK_HS=2'd0, TK_TOKEN=2'd1, TK_DATA=2'd2;

    // ---- PHY ----
    reg [1:0] dp_s, dm_s;
    reg [1:0] rx_phase;
    reg rx_line_dp, rx_line_dm;
    reg rx_last_dp, rx_last_dm;
    reg rx_active, rx_eop;
    reg [2:0] rx_eop_se0;
    reg rx_bits_en;
    reg rx_bad_stuff;
    reg rx_packet_done;
    reg [7:0] se0_count;
    reg [7:0] line_idle_count;
    reg line_sample_dp, line_sample_dm;
    reg line_reported_dp, line_reported_dm;
    reg old_se0_long;

    // ---- shared RX/TX datapath ----
    reg [7:0] shift;
    reg [2:0] bitcnt;
    reg [2:0] ones;
    reg [15:0] crc;
    reg [3:0] rxcnt;         // bytes completed this packet (0=SYNC,1=PID,..)
    reg [3:0] rx_pid;
    reg       rx_pid_ok;
    reg [7:0] tok_lo;        // first byte after PID
    reg       rx_accept;

    // ---- TX engine ----
    reg [1:0] tx_state;
    reg [1:0] tx_phase;
    reg [6:0] tx_bit_index;
    reg [3:0] tx_len;        // body bytes (data pkts); unused for token/hs
    reg [1:0] tx_crc_mode;   // 0 none, 1 crc16, 2 crc5
    reg tx_stuff;
    reg tx_line_dp, tx_line_dm;
    reg tx_done;
    reg [3:0] tx_pid;
    reg [10:0] tx_tok;
    reg tx_emit;

    // ---- transaction FSM ----
    reg [2:0] tstate;
    reg [6:0] tim;           // shared response timer (timeout 80)
    reg [3:0] pend_hs;       // device: handshake queued for data EOP
    reg [3:0] tok_pid;       // device: PID of the token that opened this op
    // host uses cmd_op/cmd_ep/cmd_data1 live: regs freeze them while busy
    reg [15:0] sof_timer;
    reg       sof_pending;

    assign tx_active = (tx_state != TX_IDLE);
    assign tx_oe = tx_active | ctrl[3] | ctrl[4];
    assign tx_dp = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b0 : tx_line_dp;
    assign tx_dm = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b1 : tx_line_dm;

    wire rx_se0 = (dp_s[1] == 0 && dm_s[1] == 0);
    wire rx_bit  = (dp_s[1] == rx_last_dp && dm_s[1] == rx_last_dm); // 1 = no NRZI change
    wire [7:0] rx_newbyte = {rx_bit, shift[7:1]};
    wire       rx_crc5 = (rx_pid[1:0] == 2'b01);
    wire [3:0] tok_ep  = {shift[2:0], tok_lo[0]}; // last token byte sits in shift at EOP
    assign tok_val = {rx_pid, tok_ep};
    assign len_val = rxcnt - 4;

    // TX emit boundaries derived from mode/len
    wire [6:0] tx_body_end = (tx_crc_mode == 2) ? 7'd27 : 7'd16 + {tx_len, 3'b0};
    wire [6:0] tx_end      = (tx_crc_mode == 0) ? 7'd16 :
                             (tx_crc_mode == 2) ? 7'd32 : 7'd32 + {tx_len, 3'b0};

    // Queue a packet for transmission.
    task tx_job;
        input [1:0] kind;
        input [3:0] pid;
        input [10:0] tok;
        input [3:0] dlen;
        begin
            tx_pid <= pid;
            tx_tok <= tok;
            tx_done <= 0;
            tx_bit_index <= 0;
            tx_phase <= 0;
            ones <= 0;
            tx_stuff <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_len <= dlen;
            crc <= 16'hffff;
            case (kind)
                TK_HS:    tx_crc_mode <= 0;
                TK_TOKEN: tx_crc_mode <= 2;
                default: begin
                    tx_crc_mode <= 1;
                    bufptr_clr <= 1;
                end
            endcase
            tx_state <= TX_BITS;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            dp_s <= 2'b11;
            dm_s <= 2'b00;
            dp_sync <= 1;
            dm_sync <= 0;
            rx_phase <= 0;
            rx_line_dp <= 1;
            rx_line_dm <= 0;
            rx_last_dp <= 1;
            rx_last_dm <= 0;
            rx_active <= 0;
            rx_eop <= 0;
            rx_eop_se0 <= 0;
            rx_bits_en <= 0;
            rx_bad_stuff <= 0;
            rx_packet_done <= 0;
            se0_count <= 0;
            line_idle_count <= 0;
            line_sample_dp <= 1;
            line_sample_dm <= 0;
            line_reported_dp <= 1;
            line_reported_dm <= 0;
            se0_long <= 0;
            old_se0_long <= 0;
            shift <= 0;
            bitcnt <= 0;
            ones <= 0;
            crc <= 0;
            rxcnt <= 0;
            rx_pid <= 0;
            rx_pid_ok <= 0;
            tok_lo <= 0;
            rx_accept <= 0;
            tx_state <= TX_IDLE;
            tx_phase <= 0;
            tx_bit_index <= 0;
            tx_len <= 0;
            tx_crc_mode <= 0;
            tx_stuff <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_done <= 0;
            tx_pid <= 0;
            tx_tok <= 0;
            tstate <= S_IDLE;
            tim <= 0;
            pend_hs <= 0;
            sof_timer <= 0;
            sof_pending <= 0;
            frame <= 0;
            busy <= 0;
            buf_we <= 0;
            buf_wdata <= 0;
            buf_inc <= 0;
            bufptr_clr <= 0;
            op_done <= 0;
            res_wr <= 0;
            res_val <= 0;

            tok_wr <= 0;

            len_wr <= 0;

            irq_set <= 0;
        end else begin
            dp_s <= {dp_s[0], dp_in};
            dm_s <= {dm_s[0], dm_in};
            dp_sync <= dp_s[1];
            dm_sync <= dm_s[1];
            tx_done <= 0;
            bufptr_clr <= 0;
            op_done <= 0;
            res_wr <= 0;
            tok_wr <= 0;
            len_wr <= 0;
            irq_set <= 0;
            rx_packet_done <= 0;
            buf_we <= 0;
            buf_inc <= 0;

            // SE0 detect / line-idle irq
            if (rx_se0) begin
                if (se0_count != 8'hff)
                    se0_count <= se0_count + 1;
                if (se0_count >= 8'd127)
                    se0_long <= 1;
            end else begin
                se0_count <= 0;
                se0_long <= 0;
            end
            if (se0_long != old_se0_long) begin
                irq_set[2] <= 1;
                old_se0_long <= se0_long;
                if (se0_long) begin
                    line_reported_dp <= 0;
                    line_reported_dm <= 0;
                end
            end
            if (tx_oe || rx_active || rx_se0) begin
                line_idle_count <= 0;
            end else if (dp_s[1] != line_sample_dp || dm_s[1] != line_sample_dm) begin
                line_sample_dp <= dp_s[1];
                line_sample_dm <= dm_s[1];
                line_idle_count <= 0;
            end else if (line_idle_count != 8'hff) begin
                line_idle_count <= line_idle_count + 1;
            end
            if (line_idle_count == 8'd127 &&
                (dp_s[1] != line_reported_dp || dm_s[1] != line_reported_dm)) begin
                line_reported_dp <= dp_s[1];
                line_reported_dm <= dm_s[1];
                irq_set[2] <= 1;
            end

            // Host SOF timer
            if (ctrl[0] && ctrl[2]) begin
                if (sof_timer == 16'd47999) begin
                    sof_timer <= 0;
                    frame <= frame + 1;
                    sof_pending <= 1;
                end else sof_timer <= sof_timer + 1;
            end else begin
                sof_timer <= 0;
                sof_pending <= 0;
            end

            // ---------------- RX bit engine ----------------
            if (tx_oe) begin
                rx_active <= 0;
                rx_phase <= 0;
                rx_line_dp <= dp_s[1];
                rx_line_dm <= dm_s[1];
                rx_last_dp <= dp_s[1];
                rx_last_dm <= dm_s[1];
            end else begin
                if (dp_s[1] != rx_line_dp || dm_s[1] != rx_line_dm) begin
                    rx_phase <= 0;
                    if (dp_s[1] != dm_s[1])
                        rx_bits_en <= 1;
                end else
                    rx_phase <= rx_phase + 1;
                rx_line_dp <= dp_s[1];
                rx_line_dm <= dm_s[1];

                if (rx_phase == 1) begin
                    if (rx_se0) begin
                        if (rx_active) begin
                            rx_eop <= 1;
                            if (rx_eop_se0 != 7)
                                rx_eop_se0 <= rx_eop_se0 + 1;
                        end
                    end else if (rx_eop) begin
                        if (dp_s[1] && !dm_s[1]) begin
                            rx_active <= 0;
                            rx_eop <= 0;
                            rx_eop_se0 <= 0;
                            rx_packet_done <= 1;
                        end
                    end else if (!rx_active) begin
                        if (rx_last_dp && !rx_last_dm && !dp_s[1] && dm_s[1]) begin
                            rx_active <= 1;
                            crc <= 16'hffff;
                            rxcnt <= 0;
                            bitcnt <= 1;
                            shift <= 0;
                            rx_last_dp <= dp_s[1];
                            rx_last_dm <= dm_s[1];
                            ones <= 0;
                            rx_bad_stuff <= 0;
                            rx_bits_en <= 0;
                        end
                    end else if (rx_bits_en) begin
                        if (rx_bit) begin
                            if (ones == 6) begin
                                rx_bad_stuff <= 1; // 1 after six 1s: stuff error
                                ones <= 0;
                            end else begin
                                shift <= rx_newbyte;
                                bitcnt <= bitcnt + 1;
                                ones <= ones + 1;
                            end
                        end else begin
                            if (ones == 6) begin
                                ones <= 0; // stuffed zero dropped
                            end else begin
                                shift <= rx_newbyte;
                                bitcnt <= bitcnt + 1;
                                ones <= 0;
                            end
                        end
                        // serial CRC over every unstuffed bit after the PID byte
                        if (rxcnt >= 2 && !(ones == 6 && !rx_bit)) begin
                            if (rx_crc5)
                                crc[4:0] <= {1'b0, crc[4:1]} ^ ((crc[0] ^ rx_bit) ? 5'h14 : 5'h0);
                            else
                                crc <= {1'b0, crc[15:1]} ^ ((crc[0] ^ rx_bit) ? 16'ha001 : 16'h0);
                        end
                        // byte completion
                        if (bitcnt == 7 && !(ones == 6 && !rx_bit)) begin
                            bitcnt <= 0;
                            if (rxcnt != 15)
                                rxcnt <= rxcnt + 1;
                            if (rxcnt == 1) begin
                                rx_pid <= rx_newbyte[3:0];
                                rx_pid_ok <= (rx_newbyte[7:4] == ~rx_newbyte[3:0]);
                            end else begin
                                if (rxcnt == 2)
                                    tok_lo <= rx_newbyte;
                                if (rxcnt >= 2 && rxcnt < 10 && rx_accept) begin
                                    buf_we <= 1;
                                    buf_wdata <= rx_newbyte;
                                end
                            end
                        end
                    end
                    if (!rx_se0) begin
                        rx_last_dp <= dp_s[1];
                        rx_last_dm <= dm_s[1];
                    end
                end
            end

            // ---------------- packet interpretation at EOP ----------------
            // Tokens: rxcnt==4, data byte count = rxcnt-4.
            if (rx_packet_done && rxcnt >= 2 && rx_pid_ok && !rx_bad_stuff) begin
                if (!ctrl[0]) begin
                    // ---------- device ----------
                    if (rx_pid[1:0] == 2'b01 && rxcnt == 4 && crc[4:0] == 5'h6) begin
                        if (rx_pid == 4'h5) begin
                            frame <= {shift[2:0], tok_lo};
                            irq_set[1] <= 1;
                        end else if (tok_lo[7:1] == addr_reg) begin
                            tok_wr <= 1;
                            tok_pid <= rx_pid;

                            if (rx_pid == 4'h9) begin
                                if (cmd_op == 2 && tok_ep == cmd_ep) begin
                                    tx_job(TK_DATA, cmd_data1 ? 4'hb : 4'h3, 11'b0, len_reg);
                                    tstate <= S_DEVACK;
                                    tim <= 0;
                                end else begin
                                    tx_job(TK_HS, (cmd_op == 3 && tok_ep == cmd_ep) ? 4'he : 4'ha, 11'b0, 4'b0);
                                    irq_set[3] <= 1;
                                end
                            end else if (rx_pid == 4'h1 || rx_pid == 4'hd) begin
                                if (rx_pid == 4'hd)
                                    pend_hs <= (cmd_op != 0) ? 4'h2 : 4'ha;
                                else if (cmd_op == 1 && (cmd_any || tok_ep == cmd_ep))
                                    pend_hs <= 4'h2;
                                else if (cmd_op == 3 && tok_ep == cmd_ep)
                                    pend_hs <= 4'he;
                                else
                                    pend_hs <= 4'ha;
                                rx_accept <= (rx_pid == 4'hd) ? (cmd_op != 0) :
                                             (cmd_op == 1 && (cmd_any || tok_ep == cmd_ep));
                                if ((rx_pid == 4'hd && cmd_op != 0) ||
                                    (rx_pid == 4'h1 && cmd_op == 1 &&
                                     (cmd_any || tok_ep == cmd_ep)))
                                    bufptr_clr <= 1;
                                if ((rx_pid == 4'hd ? (cmd_op == 0) :
                                     !(cmd_op == 1 && (cmd_any || tok_ep == cmd_ep))))
                                    irq_set[3] <= 1;
                                tstate <= S_DEVDATA;
                                tim <= 0;
                            end
                        end
                    end else if ((rx_pid == 4'h3 || rx_pid == 4'hb) && tstate == S_DEVDATA) begin
                        // data phase of OUT/SETUP
                        if (rxcnt >= 4 && crc == 16'hb001) begin
                            rx_accept <= 0;
                            if (pend_hs == 4'h2) begin
                                len_wr <= 1;
                                res_wr <= 1;
                                res_val <= {(rx_pid == 4'hb), 3'b0, tok_pid};
                                op_done <= 1;
                                irq_set[0] <= 1;
                            end
                            tx_job(TK_HS, pend_hs, 11'b0, 4'b0);
                            tstate <= S_IDLE;
                        end else begin
                            rx_accept <= 0;
                            tstate <= S_IDLE;
                        end
                    end else if (rx_pid == 4'h2 && tstate == S_DEVACK) begin
                        op_done <= 1;
                        res_wr <= 1;
                        res_val <= {1'b0, 4'b0, tok_pid};
                        irq_set[0] <= 1;
                        tstate <= S_IDLE;
                    end
                end else begin
                    // ---------- host ----------
                    if (tstate == S_WAIT && rx_pid[1:0] == 2'b10) begin
                        // ACK/NAK/STALL handshake (pid 2,a,e: class 10)
                        busy <= 0;
                        rx_accept <= 0;
                        tstate <= S_IDLE;
                        op_done <= 1;
                        res_wr <= 1;
                        res_val <= {1'b0, 4'b0, rx_pid};
                        irq_set[0] <= 1;
                    end else if (tstate == S_WAIT && (rx_pid == 4'h3 || rx_pid == 4'hb)) begin
                        if (rxcnt > 12) begin
                            busy <= 0;
                            tstate <= S_IDLE;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0, 3'd3, rx_pid};
                            irq_set[0] <= 1;
                        end else if (rxcnt >= 4 && crc == 16'hb001) begin
                            len_wr <= 1;
                            rx_accept <= 0;
                            tx_job(TK_HS, 4'h2, 11'b0, 4'b0);
                            tstate <= S_IDLE;
                            busy <= 0;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {(rx_pid == 4'hb), 3'b0, rx_pid};
                            irq_set[0] <= 1;
                        end else begin
                            busy <= 0;
                            tstate <= S_IDLE;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0, 3'd2, 4'h0};
                            irq_set[0] <= 1;
                        end
                    end
                end
            end

            // ---------------- transaction FSM ----------------
            case (tstate)
                S_IDLE: begin
                    if (ctrl[0] && !busy && sof_pending && tx_state == TX_IDLE) begin
                        tx_job(TK_TOKEN, 4'h5, frame, 4'b0);
                        sof_pending <= 0;
                    end else if (ctrl[0] && cmd_wr && cmd_op != 3 && tx_state == TX_IDLE &&
                                 (!ctrl[2] || sof_timer < 16'd46976) && !sof_pending && !busy) begin
                        busy <= 1;
                        tx_job(TK_TOKEN, cmd_op == 2 ? 4'h9 : (cmd_op == 0 ? 4'hd : 4'h1),
                               {addr_reg, cmd_ep}, 4'b0);
                        rx_accept <= 1;
                        bufptr_clr <= 1;
                    end
                    // host token done -> chain data packet or wait for IN response
                    if (busy && tx_done && tstate == S_IDLE) begin
                        if (tx_pid == 4'h9) begin
                            tstate <= S_WAIT;
                            tim <= 0;
                        end else if (tx_pid == 4'hd || tx_pid == 4'h1) begin
                            tx_job(TK_DATA, cmd_data1 ? 4'hb : 4'h3, 11'b0, len_reg);
                        end else if (tx_pid == 4'h3 || tx_pid == 4'hb) begin
                            tstate <= S_WAIT;
                            tim <= 0;
                        end
                    end
                    if (tx_done && tx_pid == 4'h5)
                        irq_set[1] <= 1;
                end
                S_WAIT, S_DEVACK: begin
                    if (rx_active || tx_state != TX_IDLE) begin
                        tim <= 0; // own transmit still in flight
                    end else if (tim == 79) begin
                        tstate <= S_IDLE;
                        if (tstate == S_WAIT) begin
                            busy <= 0;
                            rx_accept <= 0;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0, 3'd1, 4'h0};
                            irq_set[0] <= 1;
                        end
                        // device: no ACK keeps the IN endpoint armed
                    end else tim <= tim + 1;
                end
                S_DEVDATA: begin
                    if (rx_active)
                        tim <= 0;
                    else if (tim == 79) begin
                        // no data packet followed the token
                        tstate <= S_IDLE;
                        rx_accept <= 0;
                    end else tim <= tim + 1;
                end
                default: tstate <= S_IDLE;
            endcase

            // ---------------- TX bit engine ----------------
            case (tx_state)
                TX_IDLE: begin
                    tx_phase <= 0;
                    tx_line_dp <= 1;
                    tx_line_dm <= 0;
                end
                TX_BITS: begin
                    if (tx_phase == 3) begin
                        tx_phase <= 0;
                        if (tx_stuff) begin
                            tx_stuff <= 0;
                            ones <= 0;
                            tx_line_dp <= ~tx_line_dp;
                            tx_line_dm <= ~tx_line_dm;
                        end else if (tx_bit_index >= tx_end) begin
                            tx_state <= TX_EOP0;
                            tx_line_dp <= 0;
                            tx_line_dm <= 0;
                        end else begin
                            if (tx_bit_index < 8)
                                tx_emit = tx_bit_index == 7;
                            else if (tx_bit_index < 16)
                                tx_emit = (tx_bit_index[2:0] < 4) ?
                                          tx_pid[tx_bit_index[2:0]] :
                                          ~tx_pid[tx_bit_index[1:0]];
                            else if (tx_bit_index < tx_body_end) begin
                                if (tx_crc_mode == 2) begin
                                    tx_emit = tx_tok[tx_bit_index - 16];
                                    crc[4:0] <= {1'b0, crc[4:1]} ^
                                                ((crc[0] ^ tx_tok[tx_bit_index - 16]) ? 5'h14 : 5'h0);
                                end else begin
                                    tx_emit = buf_rdata[tx_bit_index[2:0]];
                                    crc <= {1'b0, crc[15:1]} ^
                                           ((crc[0] ^ buf_rdata[tx_bit_index[2:0]]) ? 16'ha001 : 16'h0);
                                end
                            end else begin
                                tx_emit = ~crc[0];
                                if (tx_crc_mode == 2)
                                    crc[4:0] <= {1'b0, crc[4:1]};
                                else
                                    crc <= {1'b0, crc[15:1]};
                            end
                            tx_bit_index <= tx_bit_index + 1;
                            // buffer pointer advances at each body byte boundary
                            if (tx_crc_mode == 1 && tx_bit_index >= 16 &&
                                tx_bit_index < tx_body_end && tx_bit_index[2:0] == 7)
                                buf_inc <= 1;
                            if (tx_emit) begin
                                ones <= ones + 1;
                                if (ones == 5) tx_stuff <= 1;
                            end else begin
                                ones <= 0;
                                tx_line_dp <= ~tx_line_dp;
                                tx_line_dm <= ~tx_line_dm;
                            end
                        end
                    end else tx_phase <= tx_phase + 1;
                end
                TX_EOP0: begin
                    tx_line_dp <= 0;
                    tx_line_dm <= 0;
                    if (tx_phase == 3) begin
                        tx_phase <= 0;
                        if (tx_bit_index == tx_end + 1) begin
                            tx_state <= TX_EOPJ;
                            tx_line_dp <= 1;
                            tx_line_dm <= 0;
                        end
                        tx_bit_index <= tx_bit_index + 1;
                    end else tx_phase <= tx_phase + 1;
                end
                TX_EOPJ: begin
                    tx_line_dp <= 1;
                    tx_line_dm <= 0;
                    if (tx_phase == 3) begin
                        tx_state <= TX_IDLE;
                        tx_phase <= 0;
                        tx_done <= 1;
                    end else tx_phase <= tx_phase + 1;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    wire _unused = &{rx_eop_se0, cmd_any, 1'b0};

endmodule
