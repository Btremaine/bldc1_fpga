`include "..\include\timescale.v"
`include "..\include\defines.v"
///////////////////////////////////////////////////////////////////////
// Company: 	Prysm Inc.
// Engineer: 	Brian Tremaine
// 
// Create Date:    01/02/2019 
// Design Name: 
// Module Name:    led_lag_comp 
// Project Name: 
// Target Devices:	Spartan 3AN XC3S50ATQ144  
// Tool versions: 
// Description: 	digital lead-lag compensation for BLDC DPLL
//		Uses signed fixed-point Q-arithmetic (.fsze)
//    #    y(z)/u(z) = Ko * (z - a) / (z - b)
//    #    y(k) = b*y(k-1) + Ko*u(k) - Ko*a*u(k-1)
//    #         = b*y(k-1) + Ko *{ u(k) - a*u(k-1)}
//    #         =     y1   + Ko *{ u(k) - y2 }
//
//      Coefficients 0< b < a < 1 (real pole & zero), 0 < Ko < (2**2)
//      Calculations of difference equation are pipelined.
//      Note if a < b < 1 this is a lag-lead compensator, a valid structure.
//
// Dependencies:	
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
///////////////////////////////////////////////////////////////////////////////
module lead_lag_comp #(parameter WIDTH_U= 18, WIDTH_ERR=22, WIDTH_LIM=16, 
                       WIDTH_FP=29, fsze = 6) (
	// input
	input sys_clk,							// system clock
	input rst,								// active high reset
	input signed [WIDTH_ERR-1:0] err,	    // signed error, Q.0
	input [WIDTH_LIM-1:0] dlim,             // err limit value
	input [fsze-1:0] af,					// af = a * (2<<fsze), Q.fsze
	input [fsze-1:0] bf,					// bf = b * (2<<fsze), Q.fsze 
	input [2+fsze-1:0] k0,                  // kf = ko * (2<<fsze), Q.fsze
	input enable,
	input process,
	// outputs
	output [WIDTH_U-1:0] uk
    );

reg signed  [WIDTH_U-1:0] result1;		    // temporary reg
reg signed  [WIDTH_U-1:0] y_old;		    // temporary reg
reg signed  [WIDTH_FP-1:0] Y1;		        // temporary reg
reg signed  [WIDTH_FP-1:0] Y2;		        // temporary reg
reg signed  [WIDTH_FP-1:0] Y3;		        // temporary reg

reg process_dly;
reg [3:0] counter;  						// state machine
reg [WIDTH_LIM-1 :0] err_lim;				// unsigned limit on error
reg signed [WIDTH_LIM-1 :0] err_lim_neg;
reg signed [WIDTH_LIM-1 :0] err_limited;
reg signed [WIDTH_LIM-1 :0] err_old;

wire signed [fsze+1:0] as;					// use signed ops
wire signed [fsze+1:0] bs;
wire signed [fsze+1:0] ks;
wire signed [WIDTH_LIM-1:0] err_lim_sgnd;

// assign
assign uk = y_old;  // final output value
assign as = af;
assign bs = bf;
assign ks = k0;
assign err_lim_sgnd = err_lim;

//
    always @ (posedge sys_clk) begin
           err_lim_neg <= ~err_lim_sgnd +1;
    end 
    always @ (posedge sys_clk) begin
           err_lim <= dlim;
    end 		
	
	always @ (posedge sys_clk, posedge rst) begin
		if(rst)
			process_dly <= 0;
		else
		   process_dly <= process;
	end
		
// ============================================================================
	always @ (posedge sys_clk, posedge rst) begin 
		if(rst) begin
				result1 <= 0;
				Y1 <= 0;
				Y2 <= 0;
				Y3 <= 0;
				y_old <= 0;
				err_old <= 0;
				counter <= 0;
		end
		else begin
			if (process & !process_dly)
				counter <= 4'h6;
			else if (counter !=0)
				counter <= counter-1;		
			if(!enable) 
			begin
				counter <= 0;
			end else begin
		    // compensation pipeline, fp math used here
		    //    y(z)/u(z) = Ko * (z - a) / (z - b)
               //    y(k) = b*y(k-1) + Ko*u(k) - Ko*a*u(k-1)
               //          = b*y(k-1) + Ko *{ u(k) - a*u(k-1)}
               //          = (b*y(k-1) + Ko *u(k)) - Ko * a*u(k-1)        
				case (counter)
					4'h6: begin
						  if(err[WIDTH_ERR-1])  // limit err (signed)
						    err_limited <= (err < err_lim_neg) ? err_lim_neg:err; // Q.0
						  else
						    err_limited <= (err > err_lim) ? err_lim : err; 
						  end
					4'h5: begin
					        Y1 <= bs * y_old + ks * err_limited; // Q.fsze
						    Y2 <= as * err_old; 	               // Q.fsze
						  end
					4'h4: begin
                                    Y3 <= (ks * Y2) >>> fsze ;		// Q.(2*fsze) -> Q.fsze
                                    end
					4'h3: begin
                                    result1 <= (Y1 - Y3) >>> fsze; // Q.fsze -> Q.0
                                   end						 					 
					4'h2: begin
							err_old <= err_limited; // Q.0
							y_old <= result1;       // Q.0
						  end
					4'h1: begin
					
					      end
					default: ;    // don't do anything
				endcase  
			end
		end
	end	
// ----------------------------------------------------------------------------------
// debug
/*	
     ila_0 lead_lag_dbg (
	.clk(sys_clk),              // 1, input wire clk

	.trig_in(result1),          // input wire trig_in 
	.trig_in_ack(trig_in_ack),  // output wire trig_in_ack 
	.probe0(err),               // 22
	.probe1(err_limited),       // 16
	.probe2(Y1),                // 29 
	.probe3(Y2),                // 29
	.probe4(Y3),                // 29
	.probe5(result1),           // 18	
	.probe6(counter)            // 4	
);	
*/
endmodule
