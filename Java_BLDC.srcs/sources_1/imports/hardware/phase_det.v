`include "..\include\timescale.v"
`include "..\include\defines.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: 	Prysm Inc.
// Engineer: 	Brian Tremaine
// 
// Create Date:    14:33:24 04/27/2016 
// Design Name: 
// Module Name:    phase_det 
// Project Name: 
// Target Devices:	Spartan 3AN XC3S50ATQ144 
// Tool versions: 
// Description: 	Phase detector measuring time between edges of system clock counts
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module phase_det #(parameter WIDTH_TMR=21, WIDTH_ERR=22) (
	// inputs
	input	clk,
	input	sync_rst_n, 
	input	ref_phase,
	input	fb_phase,
	input [WIDTH_TMR-1:0] delay_len,
	input [9:0] width_win,
	// output
	output sample_out,
	output signed [WIDTH_ERR-1:0] err,
    output pd_error,	
    output win
    );


// Parameters for PD modulo PI conversion
// 60Hz using 74.25MHz clock
// parameter N0 = 618750;
// parameter x2N0 = 1237500;
//

wire W;
wire clr_pd;
wire clr_RST;
wire set_RST;
wire clr_Q;
wire set_Q;

reg pu;
reg pd;
reg sample;
reg MIP;
reg Q;
reg S;
reg RST;
reg Wdly;
reg [WIDTH_ERR-1:0] ttimer;		// unsigned timer for measurement
reg [WIDTH_ERR-1:0] tstop;
reg [9:0]  Win_cntr;			// window counter
reg [WIDTH_TMR-1:0] delay_cntr;	// next ref_phase period
reg ref_phase_sync;
reg ref_phase_dly;
reg fb_phase_sync;
reg fb_phase_dly;
reg signed [WIDTH_ERR-1:0] Err_R;
reg signed [WIDTH_ERR-1:0] Err_mod;
reg set_ERR_dly1;
reg set_ERR_dly2;
reg set_ERR_dly3;
reg [WIDTH_ERR-1:0] no_r;
reg [WIDTH_ERR-1:0] x2no_r;

wire reset;
wire set_ERR;
wire ref_posedge;
wire fb_posedge;
wire clr_ttimer;
wire setMIP;
wire clrMIP;

// assign's go here
assign err = Err_R; // Err_mod;          
assign W = (Win_cntr != 0)? 1'b1 : 1'b0;
assign reset = ~sync_rst_n;
assign clr_pd = ((pu & pd) || RST)? 1'b1 : 1'b0;
assign set_ERR = ((pu & pd) & ~RST)? 1'b1 : 1'b0;
assign sample_out = sample;
assign ref_posedge = ref_phase_sync && ~ref_phase_dly;
assign fb_posedge = fb_phase_sync && ~fb_phase_dly;
assign clr_RST =(reset || (W & ~Wdly));
assign set_RST = (~RST & (Win_cntr == 1) & (Q==0) );
assign clr_ttimer = (~MIP & (fb_posedge || ref_posedge )) || set_RST;
assign clr_Q = (Win_cntr == 0)? 1'b1 : 1'b0;
assign set_Q = ref_phase_sync && ~ref_phase_dly;

// 11/27/18 java BLDC
// assign pd_error = RST;
assign pd_error = MIP;   // W   ( debug 'slipping' phase
assign win = W;
assign clrMIP = ((ref_posedge || fb_posedge) & MIP) || set_RST;
assign setMIP = ((ref_posedge & ~pd) || (fb_posedge & ~pu)) & ~MIP;

//
   always @ (posedge clk) begin 							// sychronise inputs
		ref_phase_sync <= ref_phase;
		ref_phase_dly <= ref_phase_sync;
		fb_phase_sync <= fb_phase;
		fb_phase_dly  <= fb_phase_sync;
		set_ERR_dly1 <= set_ERR;
		set_ERR_dly2 <= set_ERR_dly1;
		set_ERR_dly3 <= set_ERR_dly2;
		sample <= set_ERR_dly3;
		no_r <= delay_len >> 1;
		x2no_r <= delay_len;
	end
	//
	always @ (posedge clk, posedge reset) begin		// pu
		if(reset)
			pu <= 0;
		else begin
			if(ref_phase_sync && ~ref_phase_dly)
				pu <= 1'b1;
			else if (clr_pd)
				pu <= 0;
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// pd
		if(reset)
			pd <= 0;
		else begin
			if(fb_phase_sync && ~fb_phase_dly)
				pd <= 1'b1;
			else if (clr_pd)
				pd <= 0;
		end
	end		
	//
	always @ (posedge clk, posedge reset) begin		//	delay_cntr
		if(reset)
			delay_cntr <= 0;
		else begin
			if(ref_posedge)
				delay_cntr <= delay_len;
			else
				if(delay_cntr != 0)
					delay_cntr <= delay_cntr - 1;
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// ttimer
	   if(reset)
			ttimer <= 0;
		else begin
			if(MIP)
				ttimer <= ttimer + 1;
			else
			   if(clr_ttimer)
					ttimer <= 0;
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// tstop
		if(reset)
			tstop <= 0;
		else if(MIP==0)
			tstop <= ttimer;
	end
	//	
	always @ (posedge clk, posedge reset) begin		// Q
	   if(reset)
			Q <= 0;
		else begin
			if(set_Q)
				Q <= 1 ;
			else
				if(clr_Q)
					Q <= 0;
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// MIP
		if(reset)
			MIP <= 0;
		else begin
			if(clrMIP)
				MIP <= 0;
			else if (setMIP)
				MIP <= ~RST;
		end
	end
	//
	always @ (posedge clk, posedge clr_RST) begin	// RST
		if(clr_RST)
			RST <= 0;
		else begin
			if(set_RST)
				RST <= 1;
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// Win_cntr	
		if(reset)
			Win_cntr <= width_win;
		else begin
			if(delay_cntr == 1)
				Win_cntr <= width_win;
			else if(Win_cntr != 0)
				Win_cntr <= Win_cntr - 1;	
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// Wdly
		if(reset)
			Wdly <= 0;
		else
			Wdly <= W;
	end
	//		
	always @ (posedge clk, posedge reset) begin		// S
		if(reset)
			S <= 0;
		else begin
			if(~MIP) begin
				if(ref_phase_sync && ~ref_phase_dly)
					S <= 0;  // treat as negative
				else if(fb_phase_sync && ~fb_phase_dly)
					S <= 1;  // treat as positive
			end
		end
	end
	//
	always @ (posedge clk, posedge reset) begin		// Err_R
		if (reset)
			Err_R <= 0;
		else begin
			if(set_ERR_dly1) begin
				if (S==1)
					Err_R <= tstop ;
				else
					Err_R <= $signed(~tstop + 1); 		// unsigned to signed
			end
		end
	end
	// 
	always @ (posedge clk, posedge reset) begin		// Err_mod
		if(reset)
			Err_mod <= 0;
		else begin
			if (set_ERR_dly2) 
				Err_mod <= Err_R;
			if (set_ERR_dly3) begin
				if( Err_R > $signed(no_r))
					Err_mod <= $signed((Err_R - x2no_r));
				if(Err_R < $signed(~no_r+1) )
					Err_mod <= $signed(x2no_r + Err_R);	
			end
		end
	end

endmodule
