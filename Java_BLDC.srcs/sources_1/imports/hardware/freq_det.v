`include "timescale.v"
//////////////////////////////////////////////////////////////////////////////////
// Company:     Prysm Inc.
// Engineer:    Brian Tremaine
//
// Create Date:    10/30/2019
// Design Name:
// Module Name:     freq_det
// Project Name:
// Target Devices:  Artix-7 xc7a100tfgg484-3
// Tool versions:
// Description:     Frequency detector measuring period difference between two source clocks
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
//
// Additional Comments:
// * For nominal input frequency of 330Hz and a system clock of 150MHz
//   the nominal period count is 454,545. 
// * There needs to be a limit on the longest allowed period, (MAX_LIMIT). 
// * The error (difference frequency) is a signed value. 
//
//////////////////////////////////////////////////////////////////////////////////
module freq_det #(parameter WIDTH_TMR=21, WIDTH_ERR=22) (
    // inputs
    input   clk,
    input   rstn,
    input   ref_freq,
    input   fb_freq,
    // output
    output sample_out,
    output signed [WIDTH_ERR-1:0] err
    );

`ifndef	CQ
`define CQ #0.5
`endif

localparam MAX_LIMIT = 20'hFFFFF;

reg [WIDTH_TMR-1:0] fref_cnt = 0;
reg [WIDTH_TMR-1:0] fb_cnt = 0;
reg [WIDTH_ERR-1:0] Err ;
reg ready;

wire [WIDTH_TMR-1:0] fref_cnt_xq;
wire [WIDTH_TMR-1:0] fref_period_xq;
wire [WIDTH_TMR-1:0] fb_cnt_xq;
wire [WIDTH_TMR-1:0] fb_period_xq;
wire fref_pls_xq;
wire fb_pls_xq;
wire reset;

assign reset = ~rstn;
assign sample_out = ready;
	
// *************************************************************************************************************
// measure period of ref_freq	
  edge_det #("RISING")
  fref_pls_ (
      .o_xq (fref_pls_xq),   // Q
      .o    (),              // D
      .i    (ref_freq),      // CE
      .rst  (reset),         // Reset
      .clk  (clk));          // Clk

// Measure fref period
  wire[WIDTH_TMR-1:0] fref_cnt_xd = fref_pls_xq               ? 0 :
                                                      (fref_cnt_xq != MAX_LIMIT) ? fref_cnt_xq + 1 : fref_cnt_xq;
													  
  wire[WIDTH_TMR-1:0] fref_period_xd = fref_pls_xq ? fref_cnt_xq : fref_period_xq;
                                                   
  Reg #(WIDTH_TMR, 1,20'b0)  fref_cnt_xq_          (fref_cnt_xq,          fref_cnt_xd,       1'b1, reset, clk);
  Reg #(WIDTH_TMR, 1,20'b0)  fref_period_xq_       (fref_period_xq,       fref_period_xd,    1'b1, reset, clk);

// **************************************************************************************************************
// measure period of fb_freq	
  edge_det #("RISING")
  fb_pls_ (
      .o_xq (fb_pls_xq),    // Q
      .o    (),             // D
      .i    (fb_freq),      // CE
      .rst  (reset),        // Reset
      .clk  (clk));         // Clk

// Measure fb period
  wire[WIDTH_TMR-1:0] fb_cnt_xd = fb_pls_xq               ? 0 :
                                                  (fb_cnt_xq != MAX_LIMIT) ? fb_cnt_xq + 1 : fb_cnt_xq;
												  
  wire[WIDTH_TMR-1:0] fb_period_xd = fb_pls_xq ? fb_cnt_xq : fb_period_xq;
                                                   
  Reg #(WIDTH_TMR, 1,19'b0)  fb_cnt_xq_          (fb_cnt_xq,          fb_cnt_xd,       1'b1, reset, clk);
  Reg #(WIDTH_TMR, 1,20'b0)  fb_period_xq_       (fb_period_xq,       fb_period_xd,    1'b1, reset, clk);  

  // **************************************************************************************************************
	
  always @ (posedge clk)
	   if(fref_pls_xq)
	   begin
	      Err <= $signed(fref_period_xq - fb_period_xq) ;
	      ready <= 1'b1;
	   end
	   else
	      ready <= 1'b0;
	
assign err = Err;
   
endmodule
