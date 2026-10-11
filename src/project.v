/*
 * Tiny Tapeout top-level: USB FS host/device controller
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module tt_um_rhgndf_usb_hcd (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire [3:0] spi_addr, i2c_addr;
    wire [7:0] spi_wdata, i2c_wdata, spi_rdata, i2c_rdata;
    wire spi_we, spi_re, spi_active, spi_cipo;
    wire i2c_we, i2c_re, i2c_sda_oe;

    wire [5:0] ctrl;
    wire [6:0] target_addr;
    wire [3:0] cmd_ep;
    wire [1:0] cmd_op;
    wire cmd_data1, cmd_any, cmd_wr;
    wire [6:0] len_reg;
    wire [7:0] buf_rdata;
    wire [3:0] irq;
    wire irq_pin;

    wire sie_busy;
    wire dp_sync, dm_sync, se0_long;
    wire [10:0] frame;
    wire sie_buf_we, sie_buf_inc, sie_bufptr_clr;
    wire [7:0] sie_buf_wdata;
    wire sie_op_done, sie_res_wr, sie_tok_wr, sie_len_wr;
    wire [7:0] sie_res_val, sie_tok_val;
    wire [3:0] sie_irq_set;
    wire [6:0] sie_len_val;
    wire usb_tx_dp, usb_tx_dm, usb_tx_oe, usb_tx_active;
    wire sie_token;

    spi_slave u_spi (
        .clk(clk), .rst_n(rst_n),
        .sck(ui_in[0]), .copi(ui_in[1]), .cs_n(ui_in[2]),
        .cipo(spi_cipo), .active(spi_active),
        .addr(spi_addr), .wdata(spi_wdata), .we(spi_we), .re(spi_re),
        .rdata(spi_rdata)
    );

    i2c_slave u_i2c (
        .clk(clk), .rst_n(rst_n), .scl(ui_in[3]), .sda_in(uio_in[4]),
        .i2c_addr(ui_in[5:4]), .sda_oe(i2c_sda_oe),
        .addr(i2c_addr), .wdata(i2c_wdata), .we(i2c_we), .re(i2c_re),
        .rdata(i2c_rdata)
    );

    usb_regs u_regs (
        .clk(clk), .rst_n(rst_n),
        .a_active(spi_active), .a_addr(spi_addr), .a_wdata(spi_wdata),
        .a_we(spi_we), .a_re(spi_re), .a_rdata(spi_rdata),
        .b_addr(i2c_addr), .b_wdata(i2c_wdata), .b_we(i2c_we),
        .b_re(i2c_re), .b_rdata(i2c_rdata),
        .vbus(ui_in[6]), .busy(sie_busy), .dp_sync(dp_sync),
        .dm_sync(dm_sync), .se0_long(se0_long), .frame(frame),
        .ctrl(ctrl), .addr_reg(target_addr), .cmd_ep(cmd_ep),
        .cmd_op(cmd_op), .cmd_data1(cmd_data1), .cmd_any(cmd_any),
        .cmd_wr(cmd_wr), .len_reg(len_reg), .buf_rdata(buf_rdata),
        .sie_buf_we(sie_buf_we), .sie_buf_wdata(sie_buf_wdata),
        .sie_buf_inc(sie_buf_inc), .sie_bufptr_clr(sie_bufptr_clr),
        .sie_op_done(sie_op_done), .sie_res_wr(sie_res_wr),
        .sie_res_val(sie_res_val), .sie_tok_wr(sie_tok_wr),
        .sie_tok_val(sie_tok_val), .sie_len_wr(sie_len_wr),
        .sie_len_val(sie_len_val), .sie_irq_set(sie_irq_set),
        .irq(irq), .irq_pin(irq_pin)
    );

    usb_sie u_sie (
        .clk(clk), .rst_n(rst_n), .dp_in(uio_in[0]), .dm_in(uio_in[1]),
        .ctrl(ctrl), .addr_reg(target_addr), .cmd_ep(cmd_ep),
        .cmd_op(cmd_op), .cmd_data1(cmd_data1), .cmd_any(cmd_any),
        .cmd_wr(cmd_wr), .len_reg(len_reg), .buf_rdata(buf_rdata),
        .busy(sie_busy), .dp_sync(dp_sync), .dm_sync(dm_sync),
        .se0_long(se0_long), .frame(frame), .buf_we(sie_buf_we),
        .buf_wdata(sie_buf_wdata), .buf_inc(sie_buf_inc),
        .bufptr_clr(sie_bufptr_clr), .op_done(sie_op_done),
        .res_wr(sie_res_wr), .res_val(sie_res_val), .tok_wr(sie_tok_wr),
        .tok_val(sie_tok_val), .len_wr(sie_len_wr), .len_val(sie_len_val),
        .irq_set(sie_irq_set), .tx_dp(usb_tx_dp), .tx_dm(usb_tx_dm),
        .tx_oe(usb_tx_oe), .tx_active(usb_tx_active)
    );

    assign uio_out = {2'b00, spi_cipo, 1'b0, 1'b0, 1'b1, usb_tx_dm, usb_tx_dp};
    assign uio_oe  = {2'b00, spi_active, i2c_sda_oe, ctrl[0],
                      (!ctrl[0] && ctrl[1]), usb_tx_oe, usb_tx_oe};

    assign uo_out = {3'b000, usb_tx_active, dp_sync, dm_sync, ctrl[5], irq_pin};

    wire _unused = &{ena, uio_in[7:5], 1'b0};

endmodule
