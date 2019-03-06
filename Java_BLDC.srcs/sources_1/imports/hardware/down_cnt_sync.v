`include "..\include\timescale.v"
`include "..\include\defines.v"
//////////////////////////////////////////////////////////////////////////////////
// Company:		Prysm Inc
// Engineer: 	Brian Tremaine
// 
// Create Date:    13:25:19 04/27/2016 
// Design Name: 
// Module Name:    down_cnt_sync 
// Project Name: 	 Bali
// Target Devices:	Spartan 3AN XC3S50ATQ144 
// Tool versions: 
// Description:    Asynchronous load down counter. Down counts to zero and loads divide/
//                 ratio N on underflow. Output q is high for count < (N>>1) otherwise
//                 q is low.
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

module down_cnt_sync #(parameter WIDTH = 17)(
	// inputs
	input sys_clk,
	input clk_in,
	input sync_rst_n,
	input [WIDTH-1:0] N,
	// outputs
	output [WIDTH-1:0] count,
	output q_out
    );

reg [WIDTH-1:0] M;
reg  clk_dly1;
reg  clk_dly2;

// initialize
initial begin
	clk_dly1 = 0;
	clk_dly2 = 0;
	end
	
// assign
	assign q_out = (M > (N>>1)) ? 1'b1 : 1'b0;
	assign count = M;

// --------------------------------------------------
	always @(negedge sys_clk) begin
		clk_dly1 <= clk_in;
		clk_dly2 <= clk_dly1;
	end	
	
	always @(negedge sys_clk) begin
		if (~sync_rst_n) begin
			M <= N;
		end else	begin
		if( clk_dly1 && ~clk_dly2) begin
			if( M == 1)
				M <= N;
			else
				M <= M - 1'b1;
			end
		end			
	end

endmodule
