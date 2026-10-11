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
    output reg         cmd_wr,
    output reg  [3:0]  len_reg,
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
    input  wire [3:0]  sie_len_val,
    input  wire [3:0]  sie_irq_set,

    output reg  [3:0]  irq,
    output wire        irq_pin
);

    localparam ADDR_ID = 4'h0, ADDR_CTRL = 4'h1, ADDR_STATUS = 4'h2,
               ADDR_IRQ = 4'h3, ADDR_IRQ_EN = 4'h4, ADDR_ADDR = 4'h5,
               ADDR_CMD = 4'h6, ADDR_RESULT = 4'h7, ADDR_TOKEN = 4'h8,
               ADDR_LEN = 4'h9, ADDR_DATA = 4'hA, ADDR_FL = 4'hB, ADDR_FH = 4'hC;

    reg [3:0] irq_en;
    reg [7:0] result;
    reg [7:0] token;
    reg [7:0] buffer_mem [0:7];
    reg [2:0] bufptr;

    // select active port for writes / side effects
    wire        sel_a = a_active;
    wire [3:0]  waddr  = sel_a ? a_addr  : b_addr;
    wire [7:0]  wdata  = sel_a ? a_wdata : b_wdata;
    wire        wr_en  = sel_a ? a_we    : b_we;
    wire        rd_en  = sel_a ? a_re    : b_re;

    wire data_wr = wr_en && (waddr == ADDR_DATA);
    wire data_rd = rd_en && (waddr == ADDR_DATA);
    wire len_wr  = wr_en && (waddr == ADDR_LEN);
    wire buf_inc = data_wr || data_rd || sie_buf_inc || sie_buf_we;

    assign buf_rdata = buffer_mem[sie_bufptr_clr ? 3'b000 : bufptr];

    reg [7:0] a_rdata_r, b_rdata_r;
    always @* begin
        case (a_addr)
            ADDR_ID:     a_rdata_r = 8'h5B;
            ADDR_CTRL:   a_rdata_r = {2'b0, ctrl};
            ADDR_STATUS: a_rdata_r = {3'b0, vbus, busy, se0_long, dp_sync, dm_sync};
            ADDR_IRQ:    a_rdata_r = {4'b0, irq};
            ADDR_IRQ_EN: a_rdata_r = {4'b0, irq_en};
            ADDR_ADDR:   a_rdata_r = {1'b0, addr_reg};
            ADDR_CMD:    a_rdata_r = {cmd_any, cmd_data1, cmd_op, cmd_ep};
            ADDR_RESULT: a_rdata_r = result;
            ADDR_TOKEN:  a_rdata_r = token;
            ADDR_LEN:    a_rdata_r = {4'b0, len_reg};
            ADDR_DATA:   a_rdata_r = buffer_mem[sie_bufptr_clr ? 3'b000 : bufptr];
            ADDR_FL:     a_rdata_r = frame[7:0];
            ADDR_FH:     a_rdata_r = {5'b0, frame[10:8]};
            default:     a_rdata_r = 8'h00;
        endcase
        case (b_addr)
            ADDR_ID:     b_rdata_r = 8'h5B;
            ADDR_CTRL:   b_rdata_r = {2'b0, ctrl};
            ADDR_STATUS: b_rdata_r = {3'b0, vbus, busy, se0_long, dp_sync, dm_sync};
            ADDR_IRQ:    b_rdata_r = {4'b0, irq};
            ADDR_IRQ_EN: b_rdata_r = {4'b0, irq_en};
            ADDR_ADDR:   b_rdata_r = {1'b0, addr_reg};
            ADDR_CMD:    b_rdata_r = {cmd_any, cmd_data1, cmd_op, cmd_ep};
            ADDR_RESULT: b_rdata_r = result;
            ADDR_TOKEN:  b_rdata_r = token;
            ADDR_LEN:    b_rdata_r = {4'b0, len_reg};
            ADDR_DATA:   b_rdata_r = buffer_mem[sie_bufptr_clr ? 3'b000 : bufptr];
            ADDR_FL:     b_rdata_r = frame[7:0];
            ADDR_FH:     b_rdata_r = {5'b0, frame[10:8]};
            default:     b_rdata_r = 8'h00;
        endcase
    end
    assign a_rdata = a_rdata_r;
    assign b_rdata = b_rdata_r;
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
            len_reg   <= 0;
            result    <= 0;
            token     <= 0;
            bufptr    <= 0;
            cmd_wr    <= 0;
        end else begin
            cmd_wr <= 0;

            // buffer pointer
            if (sie_bufptr_clr || len_wr)
                bufptr <= buf_inc ? 1 : 0;
            else if (buf_inc)
                bufptr <= bufptr + 1;

            // buffer write port
            if (data_wr)
                buffer_mem[bufptr] <= wdata;
            else if (sie_buf_we)
                buffer_mem[sie_bufptr_clr ? 3'b000 : bufptr] <= sie_buf_wdata;

            if (wr_en) begin
                case (waddr)
                    ADDR_CTRL:   ctrl <= wdata[5:0];
                    ADDR_IRQ_EN: irq_en <= wdata[3:0];
                    ADDR_ADDR:   addr_reg <= wdata[6:0];
                    ADDR_CMD: begin
                        if (!busy) begin
                            cmd_ep    <= wdata[3:0];
                            cmd_op    <= wdata[5:4];
                            cmd_data1 <= wdata[6];
                            cmd_any   <= wdata[7];
                            cmd_wr    <= 1;
                        end
                    end
                    ADDR_LEN: len_reg <= wdata[3:0];
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
            if (sie_len_wr) len_reg <= sie_len_val;
        end
    end

    wire _unused = &{1'b0};

endmodule
