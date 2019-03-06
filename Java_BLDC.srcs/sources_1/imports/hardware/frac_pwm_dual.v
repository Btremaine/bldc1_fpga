`include "..\include\timescale.v"
`include "..\include\defines.v"
//////////////////////////////////////////////////////////////////////////////////
// Company:	Tremaine Consulting Group
// Engineer: 	Brian Tremaine
// 
// Create Date:    Jan 12, 2019 
// Design Name: 
// Module Name:    frac_pwm 
// Project Name:
// Target Devices: Artix 50T
// Tool versions: 
// Description:    fractional dual pwm
//                 Generate pwm's with constant period sys_clk/No. 
//                 Two pwm's are produced. Coarse pwm is generated 
//                 from integer bits in mf, fine pwm is generated from
//                 the fractional bits, fsze.
//                 Count is bounded by [0 No] (0-100% duty cycle)
//                 Output q1 is high for ...
//                 Output q2 is high for ...
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module frac_pwm_dual #(parameter WIDTH = 17, parameter fsze = 4) (
	// inputs
	input sys_clk,
	input sync_rst_n,
	input [WIDTH-1:0] No,			// period divider	
	input [WIDTH-1:0] N,			// default center pwm value
	input signed [WIDTH-1:0] mf,	// fractional pwm ...yyyyy.xxx
	// outputs
	output q1_out,					// pwm coarse output
	output q2_out                   // pwm fine output
    );

localparam MSB = WIDTH-1;

reg [WIDTH-1:0] ncoarse;			// coarse count
reg [WIDTH-1:0] nfine;				// fine count
reg [WIDTH-1:0] period_cnt;			// master counter
reg signed [WIDTH-1:0] mfs4;
reg signed [WIDTH-1:0] R1;

wire reset;
wire [fsze-1:0] nfrac;
wire [WIDTH-1:0] nshift;

	
// assign
assign q1_out = (period_cnt < ncoarse) ? 1'b1 : 1'b0; // coarse pwm
assign q2_out = (period_cnt < nfine) ? 1'b1 : 1'b0; // fine pwm
assign reset = ~sync_rst_n;
assign nfrac = mf[fsze-1:0] ;
assign nshift = No >>> fsze;

////// limit coarse to [0 No]
always @(posedge sys_clk) begin
     mfs4 <= mf >>> fsze;
end
always @ (posedge sys_clk) begin
     R1 <= (mf[MSB]) ? (N - (~mfs4+1)) : (N + mfs4);
end
always @ (posedge sys_clk) begin
	ncoarse <= (R1[MSB]) ? (0) : 
				           ((R1 > No) ? No: R1);
end 
always @ (posedge sys_clk) begin
    nfine = nfrac * nshift ;
end

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
    always @(negedge sys_clk, posedge reset) begin	// constant period counter
        if(reset)                                   // generates fixed frequency 
		    period_cnt <= 0;
		else begin
		    if(period_cnt !=0)
			   period_cnt <= period_cnt - 1'b1;
			else
			   period_cnt <= No;
		end
	 end
	 

endmodule
