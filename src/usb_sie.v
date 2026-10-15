/*
 * usb_sie.v - streaming USB full-speed host/device SIE.
 * Bit-level PHY (SYNC, NRZI, bit stuffing, EOP) is handled by the vendored
 * ultraembedded core_usb_fs_phy UTMI core; this module keeps the byte
 * layer: packet framing, serial CRC5/CRC16, token decode, transaction FSM.
 * No packet storage: RX bytes stream straight into the register buffer,
 * TX bytes stream out of it. All line I/O synchronous to the 48 MHz clk.
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
    input  wire [6:0]  len_reg,
    input  wire        iso,
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
    output wire [6:0]  len_val,
    output reg  [3:0]  irq_set,
    output wire        tx_dp,
    output wire        tx_dm,
    output wire        tx_oe,
    output wire        tx_active
);

    // TX job kinds
    localparam TK_HS=2'd0, TK_TOKEN=2'd1, TK_DATA=2'd2;
    // Transaction FSM states
    localparam S_IDLE=3'd0, S_WAIT=3'd1, S_DEVACK=3'd2, S_DEVDATA=3'd3;

    // ---- vendored UTMI PHY (core_usb_fs_phy) ----
    wire [7:0] utmi_data_in;
    wire       utmi_txready, utmi_rxvalid, utmi_rxactive, utmi_rxerror;
    wire [1:0] utmi_linestate;
    wire       phy_tx_dp, phy_tx_dm, phy_tx_oen;
    reg  [7:0] txd;          // utmi_data_out_i
    reg        txv;          // utmi_txvalid_i

    usb_fs_phy u_phy (
        .clk_i              (clk),
        .rst_i              (~rst_n),
        .utmi_data_out_i    (txd),
        .utmi_txvalid_i     (txv),
        .utmi_op_mode_i     (2'b00),
        .utmi_xcvrselect_i  (2'b01),
        .utmi_termselect_i  (1'b0),
        .utmi_dppulldown_i  (1'b0),
        .utmi_dmpulldown_i  (1'b0),
        // no differential receiver: rcv = J detector like usb_transceiver.v
        .usb_rx_rcv_i       (dp_in & ~dm_in),
        .usb_rx_dp_i        (dp_in),
        .usb_rx_dn_i        (dm_in),
        .usb_reset_assert_i (ctrl[3]),        // FORCE_SE0 -> STATE_TX_RST
        .utmi_data_in_o     (utmi_data_in),
        .utmi_txready_o     (utmi_txready),
        .utmi_rxvalid_o     (utmi_rxvalid),
        .utmi_rxactive_o    (utmi_rxactive),
        .utmi_rxerror_o     (utmi_rxerror),
        .utmi_linestate_o   (utmi_linestate),
        .usb_tx_dp_o        (phy_tx_dp),
        .usb_tx_dn_o        (phy_tx_dm),
        .usb_tx_oen_o       (phy_tx_oen),
        .usb_reset_detect_o (),
        .usb_en_o           ()
    );

    wire phy_oe = ~phy_tx_oen;
    // blank our receiver while we transmit or while FORCE_K is asserted
    wire rx_blank  = phy_oe | ctrl[4];
    wire rxact     = utmi_rxactive & ~rx_blank;
    wire rxv       = utmi_rxvalid & ~rx_blank;

    assign tx_oe = phy_oe | ctrl[3] | ctrl[4];
    assign tx_dp = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b0 : phy_tx_dp;
    assign tx_dm = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b1 : phy_tx_dm;
    assign tx_active = phy_oe & ~ctrl[3];

    // ---- our own line state for STATUS/IRQ.LINE (unchanged semantics) ----
    reg [1:0] dp_s, dm_s;
    reg [7:0] se0_count;
    reg [7:0] line_idle_count;
    reg line_sample_dp, line_sample_dm;
    reg line_reported_dp, line_reported_dm;
    reg old_se0_long;

    // ---- shared RX/TX datapath ----
    reg [15:0] crc;
    reg [10:0] cf;           // serial CRC feed shift register (LSB first)
    reg [3:0]  cfcnt;        // bits left to feed
    reg        cf5;          // feed mode: 1 = crc5, 0 = crc16
    reg [6:0]  rxcnt;        // rxvalid bytes this packet (0=PID not yet,1=PID,2..=body)
    reg [3:0]  rx_pid;
    reg        rx_pid_ok;
    reg [7:0]  rx_byte;      // last received byte
    reg [7:0]  tok_lo;       // first byte after PID
    reg        rx_accept;
    reg        rx_err;       // utmi_rxerror seen during packet
    reg        rxact_d;
    reg        pend_eval;    // EOP seen, waiting for CRC feed to drain
    reg        rx_packet_done;

    // ---- TX byte sequencer ----
    reg        tx_ours;      // a tx_job-issued packet is in flight
    reg        phy_oe_d;
    reg        tx_done;
    reg [3:0]  tx_pid;
    reg [10:0] tx_tok;
    reg [1:0]  tx_kind;
    reg [7:0]  tx_rem;       // bytes remaining after the PID byte
    reg [1:0]  tx_crc_mode;  // 0 none, 1 crc16, 2 crc5

    // ---- transaction FSM ----
    reg [2:0] tstate;
    reg [6:0] tim;           // shared response timer (timeout 80)
    reg [3:0] pend_hs;       // device: handshake queued for data EOP
    reg [3:0] tok_pid;       // device: PID of the token that opened this op
    reg       pend_iso;      // device: current op is isochronous (no handshake)
    // host uses cmd_op/cmd_ep/cmd_data1 live: regs freeze them while busy
    reg [15:0] sof_timer;
    reg        sof_pending;

    wire rx_se0 = (dp_s[1] == 0 && dm_s[1] == 0);
    wire       rx_crc5 = (rx_pid[1:0] == 2'b01);
    wire [3:0] tok_ep  = {rx_byte[2:0], tok_lo[0]}; // last token byte in rx_byte
    assign tok_val = {rx_pid, tok_ep};
    assign len_val = rxcnt - 3;

    // Queue a packet for transmission.
    task tx_job;
        input [1:0] kind;
        input [3:0] pid;
        input [10:0] tok;
        input [6:0] dlen;
        begin
            tx_kind <= kind;
            tx_pid <= pid;
            tx_tok <= tok;
            txd <= {~pid, pid};
            txv <= 1;
            tx_ours <= 1;
            tx_done <= 0;
            crc <= 16'hffff;
            case (kind)
                TK_HS:    begin tx_crc_mode <= 0; tx_rem <= 0;            end
                TK_TOKEN: begin tx_crc_mode <= 2; tx_rem <= 8'd2;         end
                default:  begin tx_crc_mode <= 1; tx_rem <= {1'b0, dlen} + 8'd2;
                                bufptr_clr <= 1;                          end
            endcase
        end
    endtask

    // next TX byte after the one just accepted (tx_rem is pre-decrement count)
    wire [7:0] tx_next =
        (tx_kind == TK_TOKEN) ? ((tx_rem == 8'd2) ? tx_tok[7:0]
                                                 : {~crc[4:0], tx_tok[10:8]}) :
        /* TK_DATA */         ((tx_rem > 8'd2)   ? buf_rdata :
                              (tx_rem == 8'd2)   ? ~crc[7:0] : ~crc[15:8]);

    always @(posedge clk) begin
        if (!rst_n) begin
            dp_s <= 2'b11;
            dm_s <= 2'b00;
            dp_sync <= 1;
            dm_sync <= 0;
            se0_count <= 0;
            line_idle_count <= 0;
            line_sample_dp <= 1;
            line_sample_dm <= 0;
            line_reported_dp <= 1;
            line_reported_dm <= 0;
            se0_long <= 0;
            old_se0_long <= 0;
            crc <= 0;
            cf <= 0;
            cfcnt <= 0;
            cf5 <= 0;
            rxcnt <= 0;
            rx_pid <= 0;
            rx_pid_ok <= 0;
            rx_byte <= 0;
            tok_lo <= 0;
            rx_accept <= 0;
            rx_err <= 0;
            rxact_d <= 0;
            pend_eval <= 0;
            rx_packet_done <= 0;
            tx_ours <= 0;
            phy_oe_d <= 0;
            txv <= 0;
            txd <= 0;
            tx_pid <= 0;
            tx_tok <= 0;
            tx_kind <= 0;
            tx_rem <= 0;
            tx_crc_mode <= 0;
            tx_done <= 0;
            tstate <= S_IDLE;
            tim <= 0;
            pend_hs <= 0;
            pend_iso <= 0;
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
            if (tx_oe || rxact || rx_se0) begin
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

            // ---------------- serial CRC feed ----------------
            if (cfcnt != 0) begin
                if (cf5)
                    crc[4:0] <= {1'b0, crc[4:1]} ^ ((crc[0] ^ cf[0]) ? 5'h14 : 5'h0);
                else
                    crc <= {1'b0, crc[15:1]} ^ ((crc[0] ^ cf[0]) ? 16'ha001 : 16'h0);
                cf <= {1'b0, cf[10:1]};
                cfcnt <= cfcnt - 1;
            end

            // ---------------- UTMI RX byte layer ----------------
            rxact_d <= rxact;
            phy_oe_d <= phy_oe;
            if (rxact && !rxact_d) begin
                // start of packet (first byte after SYNC)
                rxcnt <= 0;
                crc <= 16'hffff;
                cfcnt <= 0;
                rx_err <= 0;
                rx_pid_ok <= 0;
            end
            if (rxact && utmi_rxerror)
                rx_err <= 1;
            if (rxv) begin
                rx_byte <= utmi_data_in;
                if (rxcnt == 0) begin
                    // PID byte
                    rx_pid <= utmi_data_in[3:0];
                    rx_pid_ok <= (utmi_data_in[7:4] == ~utmi_data_in[3:0]);
                    rxcnt <= 1;
                end else begin
                    if (rxcnt != 127)
                        rxcnt <= rxcnt + 1;
                    if (rxcnt == 1)
                        tok_lo <= utmi_data_in;
                    if (rxcnt >= 1 && rxcnt < 65 && rx_accept) begin
                        buf_we <= 1;
                        buf_wdata <= utmi_data_in;
                    end
                    // CRC covers every byte after the PID
                    cf <= {3'b0, utmi_data_in};
                    cfcnt <= 8;
                    cf5 <= rx_crc5;
                end
            end
            // end of packet: rxactive falls; evaluate once the CRC feed has
            // drained (the last byte's bits finish a few clocks later)
            if (rxact_d && !rxact)
                pend_eval <= 1;
            if (pend_eval && cfcnt == 0 && !rxv) begin
                pend_eval <= 0;
                rx_packet_done <= 1;
            end

            // ---------------- UTMI TX byte layer ----------------
            // tx_done when the PHY finishes driving (our packets only)
            if (phy_oe_d && !phy_oe && tx_ours) begin
                tx_done <= 1;
                tx_ours <= 0;
            end
            if (txv && utmi_txready) begin
                if (tx_rem == 0) begin
                    txv <= 0;
                end else begin
                    tx_rem <= tx_rem - 1;
                    txd <= tx_next;
                    // feed body bits into the CRC
                    if (tx_crc_mode == 2 && tx_rem == 8'd2) begin
                        cf <= tx_tok;
                        cfcnt <= 11;
                        cf5 <= 1;
                    end else if (tx_crc_mode == 1 && tx_rem > 8'd2) begin
                        cf <= {3'b0, buf_rdata};
                        cfcnt <= 8;
                        cf5 <= 0;
                    end
                    if (tx_crc_mode == 1 && tx_rem > 8'd2)
                        buf_inc <= 1;
                end
            end

            // ---------------- packet interpretation at EOP ----------------
            // Tokens end with rxcnt==3 (PID + 2 bytes); data length = rxcnt-3.
            if (rx_packet_done && rxcnt >= 1 && rx_pid_ok && !rx_err) begin
                if (!ctrl[0]) begin
                    // ---------- device ----------
                    if (rx_pid[1:0] == 2'b01 && rxcnt == 3 && crc[4:0] == 5'h6) begin
                        if (rx_pid == 4'h5) begin
                            frame <= {rx_byte[2:0], tok_lo};
                            irq_set[1] <= 1;
                        end else if (tok_lo[7:1] == addr_reg) begin
                            tok_wr <= 1;
                            tok_pid <= rx_pid;
                            pend_iso <= iso && tok_ep == cmd_ep &&
                                        (rx_pid == 4'h1 || rx_pid == 4'h9);

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
                        if (rxcnt >= 3 && crc == 16'hb001) begin
                            rx_accept <= 0;
                            if (pend_hs == 4'h2) begin
                                len_wr <= 1;
                                res_wr <= 1;
                                res_val <= {(rx_pid == 4'hb), 3'b0, tok_pid};
                                op_done <= 1;
                                irq_set[0] <= 1;
                            end
                            if (!pend_iso)
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
                        if (rxcnt > 67) begin
                            busy <= 0;
                            tstate <= S_IDLE;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0, 3'd3, rx_pid};
                            irq_set[0] <= 1;
                        end else if (rxcnt >= 3 && crc == 16'hb001) begin
                            len_wr <= 1;
                            rx_accept <= 0;
                            if (!(iso && cmd_op == 2'b10))
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
                    if (ctrl[0] && !busy && sof_pending && !phy_oe && !txv) begin
                        tx_job(TK_TOKEN, 4'h5, frame, 4'b0);
                        sof_pending <= 0;
                    end else if (ctrl[0] && cmd_wr && cmd_op != 3 && !phy_oe && !txv &&
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
                            if (iso && cmd_op == 2'b01) begin
                                // iso OUT: no handshake expected
                                busy <= 0;
                                op_done <= 1;
                                res_wr <= 1;
                                res_val <= 8'h00;
                                irq_set[0] <= 1;
                            end else begin
                                tstate <= S_WAIT;
                                tim <= 0;
                            end
                        end
                    end
                    if (tx_done && tx_pid == 4'h5)
                        irq_set[1] <= 1;
                end
                S_WAIT, S_DEVACK: begin
                    // iso device IN: complete when the data packet is sent
                    if (tstate == S_DEVACK && pend_iso && tx_done) begin
                        op_done <= 1;
                        res_wr <= 1;
                        res_val <= {1'b0, 4'b0, tok_pid};
                        irq_set[0] <= 1;
                        tstate <= S_IDLE;
                    end else if (rxact || phy_oe || txv) begin
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
                    if (rxact)
                        tim <= 0;
                    else if (tim == 79) begin
                        // no data packet followed the token
                        tstate <= S_IDLE;
                        rx_accept <= 0;
                    end else tim <= tim + 1;
                end
                default: tstate <= S_IDLE;
            endcase
        end
    end

    wire _unused = &{cmd_any, utmi_linestate, 1'b0};

endmodule
