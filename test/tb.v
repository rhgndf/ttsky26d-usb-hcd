`default_nettype none
`timescale 1ns / 1ps

module tb ();
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
  end

  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] model_in;
  reg [1:0] model_usb_oe;
  reg model_usb_dp;
  reg model_usb_dm;
  reg model_fs_pullup;
  reg model_ls_pullup;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
  wire [7:0] uio_in;
  reg [31:0] contention_count;

  wire dp_bus = uio_oe[0] ? uio_out[0] :
                model_usb_oe[0] ? model_usb_dp :
                (uio_oe[2] && uio_out[2]) || model_fs_pullup;
  wire dm_bus = uio_oe[1] ? uio_out[1] :
                model_usb_oe[1] ? model_usb_dm : model_ls_pullup;
  wire sda_bus = model_in[4] && !(uio_oe[4] && uio_out[4] == 1'b0);

  assign uio_in = {model_in[7:5], sda_bus, 1'b0, 1'b0, dm_bus, dp_bus};

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_rhgndf_usb_hcd user_project (
`ifdef GL_TEST
      .VPWR(VPWR), .VGND(VGND),
`endif
      .ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
      .uio_out(uio_out), .uio_oe(uio_oe), .ena(ena),
      .clk(clk), .rst_n(rst_n)
  );

  always @(posedge clk) begin
    if (!rst_n)
      contention_count <= 0;
    else if ((uio_oe[0] && model_usb_oe[0] && uio_out[0] != model_usb_dp) ||
             (uio_oe[1] && model_usb_oe[1] && uio_out[1] != model_usb_dm))
      contention_count <= contention_count + 1;
  end
endmodule
