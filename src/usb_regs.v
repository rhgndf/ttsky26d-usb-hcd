/*
 * usb_regs.v - register file for USB FS host/device controller
 * Two register-bus ports (SPI port A, I2C port B); port A wins while active.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module usb_regs (
    input  wire        clk,
    input  wire        rst_n,

    // register bus port A (SPI)
    input  wire        a_active,
    input  wire [3:0]  a_addr,
    input  wire [7:0]  a_wdata,
    input  wire        a_we,
    input  wire        a_re,
    output wire [7:0]  a_rdata,

    // register bus port B (I2C)
    input  wire [3:0]  b_addr,
    input  wire [7:0]  b_wdata,
    input  wire        b_we,
    input  wire        b_re,
    output wire [7:0]  b_rdata,

    // status inputs
    input  wire        vbus,
    input  wire        busy,
    input  wire        dp_sync,
    input  wire        dm_sync,
    input  wire        se0_long,
    input  wire [10:0] frame,

    // SIE interface
    output reg  [5:0]  ctrl,
    output reg  [6:0]  addr_reg,
    output reg  [3:0]  cmd_ep,
    output reg  [1:0]  cmd_op,
    output reg         cmd_data1,
    output reg         cmd_any,
    output reg         cmd_pend,
    input  wire        sie_cmd_ack,
    output wire [6:0]  len_reg,
    output wire        iso,
    output wire [7:0]  buf_rdata,
    input  wire        sie_buf_we,
    input  wire [7:0]  sie_buf_wdata,
    input  wire        sie_buf_inc,
    input  wire        sie_bufptr_clr,
    input  wire        sie_op_done,
    input  wire        sie_res_wr,
    input  wire [7:0]  sie_res_val,
    input  wire        sie_tok_wr,
    input  wire [7:0]  sie_tok_val,
    input  wire        sie_len_wr,
    input  wire [6:0]  sie_len_val,
    input  wire [3:0]  sie_irq_set,

    output reg  [3:0]  irq,
    output wire        irq_pin
);

    localparam ADDR_ID = 4'h0, ADDR_CTRL = 4'h1, ADDR_STATUS = 4'h2,
               ADDR_IRQ = 4'h3, ADDR_IRQ_EN = 4'h4, ADDR_ADDR = 4'h5,
               ADDR_CMD = 4'h6, ADDR_RESULT = 4'h7, ADDR_TOKEN = 4'h8,
               ADDR_LEN = 4'h9, ADDR_DATA = 4'hA, ADDR_FL = 4'hB,
               ADDR_FH = 4'hC, ADDR_BUF = 4'hD, ADDR_HBUF = 4'hE;

    reg [3:0] irq_en;
    reg [7:0] result;
    reg [7:0] token;

    // BUF: [1] SBUF (SIE buffer) [2] AUTO_SWAP [3] ISO
    reg       sbuf, auto_swap;
    reg       iso_reg;
    // HBUF: [0] buffer accessed by DATA/LEN over SPI/I2C
    reg       hbuf;
    reg [6:0] len0, len1;
    reg [5:0] sptr, hptr;

    assign iso     = iso_reg;
    assign len_reg = sbuf ? len1 : len0;

    // 128-byte packet buffer: tnt's rf_top register file, 32 words x 32 bits;
    // buffer 0 is the low half of each word, buffer 1 the high half.
    // Synchronous read: both read ports are addressed with the next-state
    // word address so the outputs always hold the word at the pointer.
    reg  [31:0] fw_data;
    reg  [4:0]  fw_addr;
    reg         fw_valid;
    wire [31:0] ra_data, rb_data;

    // select active port for writes / side effects
    wire        sel_a = a_active;
    wire [3:0]  waddr  = sel_a ? a_addr  : b_addr;
    wire [7:0]  wdata  = sel_a ? a_wdata : b_wdata;
    wire        wr_en  = sel_a ? a_we    : b_we;
    wire        rd_en  = sel_a ? a_re    : b_re;

    wire data_wr = wr_en && (waddr == ADDR_DATA);
    wire data_rd = rd_en && (waddr == ADDR_DATA);
    wire len_wr  = wr_en && (waddr == ADDR_LEN);
    wire hbuf_wr = wr_en && (waddr == ADDR_HBUF);

    // ---- pointers ----
    wire [5:0] sptr_next = sie_bufptr_clr ? 6'd0 :
                           (sie_buf_inc || sie_buf_we) ? sptr + 6'd1 : sptr;

    // one-entry pending host write: a host DATA write landing in the same
    // cycle as an SIE write is deferred by one cycle; enough because SIE
    // writes are >=32 clocks apart and host DATA accesses >= ~96.
    reg       hpend;
    reg [7:0] hpend_byte;
    wire      host_issue = !sie_buf_we && (data_wr || hpend);
    wire [7:0] host_byte = hpend ? hpend_byte : wdata;

    wire hptr_clr = len_wr || hbuf_wr || (sie_len_wr && sbuf == hbuf);
    wire [5:0] hptr_next = hptr_clr ? 6'd0 :
                           (data_rd || host_issue) ? hptr + 6'd1 : hptr;

    // ---- write path (RMW into the full 32-bit word) ----
    wire [4:0] w_word  = sie_buf_we ? sptr[5:1] : hptr[5:1];
    wire       w_buf   = sie_buf_we ? sbuf : hbuf;
    wire       w_byte  = sie_buf_we ? sptr[0] : hptr[0];
    wire [7:0] w_data8 = sie_buf_we ? sie_buf_wdata : host_byte;
    wire       buf_we  = sie_buf_we || host_issue;

    wire [31:0] word_a = (fw_valid && fw_addr == sptr[5:1]) ? fw_data : ra_data;
    wire [31:0] word_b = (fw_valid && fw_addr == hptr[5:1]) ? fw_data : rb_data;
    wire [31:0] cur_word = sie_buf_we ? word_a : word_b;
    wire [31:0] new_word = cur_word & ~(32'hff << ({w_buf, w_byte} * 8))
                           | ({24'b0, w_data8} << ({w_buf, w_byte} * 8));

    rf_top i_rf (
        .w_data  (new_word),
        .w_addr  (w_word),
        .w_ena   (buf_we),
        .ra_addr (sptr_next[5:1]),
        .rb_addr (hptr_next[5:1]),
        .ra_data (ra_data),
        .rb_data (rb_data),
        .clk     (clk)
    );

    // SIE read data: byte of the port-A word selected by SBUF/sptr[0]
    assign buf_rdata = word_a >> ({sbuf, sptr[0]} * 8);
    // host DATA read: byte of the port-B word selected by HBUF/hptr[0]
    wire [7:0] host_rdata = word_b >> ({hbuf, hptr[0]} * 8);

    // single read port muxed by interface: SPI when CS_N low, else I2C
    reg [7:0] rdata_r;
    wire [3:0] raddr = sel_a ? a_addr : b_addr;
    always @* begin
        case (raddr)
            ADDR_ID:     rdata_r = 8'h5B;
            ADDR_CTRL:   rdata_r = {2'b0, ctrl};
            ADDR_STATUS: rdata_r = {3'b0, vbus, busy | cmd_pend, se0_long, dp_sync, dm_sync};
            ADDR_IRQ:    rdata_r = {4'b0, irq};
            ADDR_IRQ_EN: rdata_r = {4'b0, irq_en};
            ADDR_ADDR:   rdata_r = {1'b0, addr_reg};
            ADDR_CMD:    rdata_r = {cmd_any, cmd_data1, cmd_op, cmd_ep};
            ADDR_RESULT: rdata_r = result;
            ADDR_TOKEN:  rdata_r = token;
            ADDR_LEN:    rdata_r = {1'b0, hbuf ? len1 : len0};
            ADDR_DATA:   rdata_r = host_rdata;
            ADDR_FL:     rdata_r = frame[7:0];
            ADDR_FH:     rdata_r = {5'b0, frame[10:8]};
            ADDR_BUF:    rdata_r = {4'b0, iso_reg, auto_swap, sbuf, 1'b0};
            ADDR_HBUF:   rdata_r = {7'b0, hbuf};
            default:     rdata_r = 8'h00;
        endcase
    end
    assign a_rdata = rdata_r;
    assign b_rdata = rdata_r;
    assign irq_pin = |(irq & irq_en);

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl      <= 0;
            addr_reg  <= 0;
            irq       <= 0;
            irq_en    <= 0;
            cmd_ep    <= 0;
            cmd_op    <= 0;
            cmd_data1 <= 0;
            cmd_any   <= 0;
            len0      <= 0;
            len1      <= 0;
            result    <= 0;
            token     <= 0;
            sbuf      <= 0;
            auto_swap <= 0;
            iso_reg   <= 0;
            hbuf      <= 0;
            sptr      <= 0;
            hptr      <= 0;
            hpend     <= 0;
            hpend_byte <= 0;
            fw_valid  <= 0;
            cmd_pend  <= 0;
        end else begin

            sptr <= sptr_next;
            hptr <= hptr_next;

            // deferred host DATA write on SIE-write collision
            if (sie_buf_we && data_wr) begin
                hpend <= 1;
                hpend_byte <= wdata;
            end else if (hpend && !sie_buf_we) begin
                hpend <= 0;
            end

            // write forwarding for read-during-write (macro uncharacterized)
            fw_valid <= buf_we;
            if (buf_we) begin
                fw_addr <= w_word;
                fw_data <= new_word;
            end

            if (wr_en) begin
                case (waddr)
                    ADDR_CTRL:   ctrl <= wdata[5:0];
                    ADDR_IRQ_EN: irq_en <= wdata[3:0];
                    ADDR_ADDR:   addr_reg <= wdata[6:0];
                    ADDR_CMD: begin
                        if (ctrl[0]) begin
                            // host: queue the op until the SIE can start it
                            if (!busy && !cmd_pend && wdata[5:4] != 2'b11) begin
                                cmd_ep    <= wdata[3:0];
                                cmd_op    <= wdata[5:4];
                                cmd_data1 <= wdata[6];
                                cmd_any   <= wdata[7];
                                cmd_pend  <= 1;
                            end
                        end else if (!busy) begin
                            cmd_ep    <= wdata[3:0];
                            cmd_op    <= wdata[5:4];
                            cmd_data1 <= wdata[6];
                            cmd_any   <= wdata[7];
                        end
                    end
                    ADDR_LEN: begin
                        if (hbuf) len1 <= wdata[6:0];
                        else      len0 <= wdata[6:0];
                    end
                    ADDR_BUF: begin
                        sbuf      <= wdata[1];
                        auto_swap <= wdata[2];
                        iso_reg   <= wdata[3];
                    end
                    ADDR_HBUF: hbuf <= wdata[0];
                    default: ;
                endcase
            end

            // IRQ: W1C with set-wins
            irq <= (wr_en && waddr == ADDR_IRQ ? (irq & ~wdata[3:0]) : irq)
                   | sie_irq_set;

            if (sie_op_done)
                cmd_op <= 0;

            if (sie_res_wr) result <= sie_res_val;
            if (sie_tok_wr) token  <= sie_tok_val;
            // sie_len_wr arrives the same cycle as op_done: the LEN write
            // lands in the OLD SBUF (nonblocking), before any auto-swap.
            if (sie_len_wr) begin
                if (sbuf) len1 <= sie_len_val;
                else      len0 <= sie_len_val;
            end

            // AUTO_SWAP: toggle SBUF on a completed non-NAK/non-STALL op
            if (sie_op_done && sie_res_wr && auto_swap &&
                sie_res_val[6:4] == 3'b0 &&
                sie_res_val[3:0] != 4'ha && sie_res_val[3:0] != 4'he)
                sbuf <= ~sbuf;

            // pending host op: the SIE started it, or host mode left
            if (sie_cmd_ack || !ctrl[0])
                cmd_pend <= 0;
        end
    end


endmodule
