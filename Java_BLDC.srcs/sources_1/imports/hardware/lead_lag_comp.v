`include "timescale.v"
///////////////////////////////////////////////////////////////////////
// Company:     Prysm Inc.
// Engineer:    Brian Tremaine
//
// Create Date:    01/07/2019 
// Design Name:
// Module Name:    led_lag_comp
// Project Name:
// Target Devices:  Spartan 3AN XC3S50ATQ144
// Tool versions:
// Description:     digital compensation for BLDC DPLL
//                  Cascade integrator with lead-lag 
//                  Uses signed fixed-point Q-arithmetic (.fsze)
//
//    				y(z)/u(z) = Ks * Ki * (z - a) / [(z - b)(z-1)]
//    
//    				implemented in cascade as:
//    					y1(k) = y1(k-1) + u(k)    
//    					y2(k) = b*y2(k-1) + y1(k) - a*y1(k-1)
//                      y3(k) = K1*Ks*y2(k)
//
//    	Ki is Q.(fsze+3). Cascade unity integrator as first block and apply
//      gains Ki and Ks on final output to prevent roundoff truncation to 0.
//      Coefficients 0< b < a < 1 (two real poles & zero), 0 < Ks,Ki < (2**2)
//      Calculations of difference equation are pipelined.
//
// Dependencies:
//
// Revision:  7/27/19 modify to add 2nd lag
//            10/21/2019 add sample_2, signal to signify data available at output
//            12/30/19  change integrator from parallel to cascade block to reduce HF noise
//            01/07/19  re-arrange calculations to prevent truncation to zero on integrator.
// Revision 0.01 - File Created
// Additional Comments:
//
///////////////////////////////////////////////////////////////////////////////
module lead_lag_comp #(parameter WIDTH_U=19, WIDTH_ERR=22, WIDTH_LIM=18,
                       WIDTH_FP=29, fsze=6) (
    // input
    input sys_clk,                          // system clock
    input rst,                              // active high reset
    input signed [WIDTH_ERR-1:0] err,       // signed error, Q.0
    input [WIDTH_LIM-1:0] dlim,             // err limit value
    input [WIDTH_FP-1:0] intlim,            // integrator limit
    input [15:0] af,                        // af = a * (2<<fsze), Q.fsze
    input [15:0] bf,                        // bf = b * (2<<fsze), Q.fsze
    input [15:0] cf,                        // cf = c * (2<<fsze), Q.fsze (** not used **)
    input [15:0] k0,                        // kf = ko * (2<<fsze), Q.fsze
    input [15:0] kif,                       // kif = ki * (2<<fsze), Q.fsze
    input enable,
    input process_in,
	input [15:0] offset,                    // output offset error, unsigned
    // outputs
    output signed [WIDTH_U-1:0] uk,         // signed data 
    output [15:0] outp,                     // unsigned 16-bit data for dshot
    output sample_out
    );

localparam MSB_FP = WIDTH_FP-1;             // msb of fixed point
localparam MSB_LIM = WIDTH_LIM-1;

reg [15:0]  outp2;
reg signed  [WIDTH_FP-1:0] result1;         // temporary reg
reg signed  [WIDTH_FP-1:0] y_old;           // temporary reg
reg signed  [WIDTH_U-1:0] y_out;            // temporary reg
reg signed  [WIDTH_U-1:0] y_off;            // temporary reg

reg signed  [WIDTH_FP-1:0] Y1;              // temporary reg
reg signed  [WIDTH_FP-1:0] Y2;              // temporary reg
reg signed  [WIDTH_FP-1:0] Y3;              // temporary reg
reg signed  [WIDTH_FP-1:0] Yo;              // temporary reg
reg signed  [WIDTH_FP-1:0] Yi;              // integrator raw
reg signed  [WIDTH_FP-1:0] Ys;              // integrator saturated
reg signed  [WIDTH_FP-1:0] temp;            // temporary register
reg signed  [WIDTH_FP-1:0] int_lim_neg;     //
reg signed  [WIDTH_FP-1 :0] err_old;
reg [WIDTH_FP-1:0] integ_lim;               // unsigned limit on integrator

reg process_dly;
reg [4:0] counter;                          // state machine
reg [WIDTH_LIM-1 :0] err_lim;               // unsigned limit on error
reg signed [WIDTH_LIM-1 :0] err_lim_neg;
reg signed [WIDTH_LIM-1 :0] err_limited;


reg undrflow;
reg ovrflow;
reg extra;
reg sample_1;

wire signed [fsze+1:0] as;                  // use signed ops
wire signed [fsze+1:0] bs;
wire signed [fsze+1:0] cs;
wire signed [fsze+1:0] ks;
wire signed [fsze+3:0] ki;

wire signed [WIDTH_LIM-1:0] err_lim_sgnd;
wire signed [WIDTH_FP-1:0] int_lim_sgnd;

// assign
assign uk = y_out;     // final output value
assign as = af;
assign bs = bf;
assign cs = cf;
assign ks = k0;
assign ki = kif;
assign err_lim_sgnd = err_lim;
assign int_lim_sgnd = integ_lim;
assign sample_out = sample_1;
assign outp = outp2;

//
    always @ (posedge sys_clk) begin
           err_lim_neg <= ~err_lim_sgnd +1;
           int_lim_neg <= ~int_lim_sgnd + 1;
    end
    always @ (posedge sys_clk) begin
           err_lim <= dlim;
           integ_lim <= intlim;
    end

    always @ (posedge sys_clk, posedge rst) begin
        if(rst)
            process_dly <= 0;
        else
           process_dly <= process_in;
    end

// ============================================================================
    always @ (posedge sys_clk, posedge rst) begin
        if(rst | !enable) begin
                result1 <= 0;
                Y1 <= 0;
                Y2 <= 0;
                Y3 <= 0;
                Yi <= 0;
                Ys <= 0;
                y_old <= 0;
                y_out <= 0;
                y_off <= 0;
                err_old <= 0;
                extra <= 0;
                temp <= 0;
                ovrflow <= 0;
                undrflow <= 0;
                err_limited <= 0;
                counter <= 0;
        end
        else begin // computation
            if (process_in & !process_dly)
            begin
                counter <= 5'd19;
                sample_1 <= 0;
            end
            else if (counter !=0)
                counter <= counter-1;
            begin // state machine
            // compensation pipeline, fp math used here
            // add integrator in cascade
			//    	y(z)/u(z) = Ks * Ki * (z - a) / [(z - b)(z-1)]
			//    
			//    	implemented in cascade as:
			//    		y1(k) = y1(k-1) +  {lim} u(k-1)    
			//    		y2(k) = y1(k) + b*y1(k-1) - a*y2(k)
			//          y3(k) = Ki*Ks*y2(k)
			//
                case (counter)
				    // ******* integrator with input limits and saturation *******
                    5'd19: begin
                           if(err[WIDTH_ERR-1])  // limit err (signed)
                            err_limited <= (err < err_lim_neg) ? err_lim_neg:err; // Q.0
                           else
                            err_limited <= (err > err_lim) ? err_lim : err;
                           end
                    5'd18: begin
                           // temp == extend err_limited to width WIDTH_FP
                           temp <= {{(WIDTH_FP-WIDTH_LIM){err_limited[MSB_LIM]}}, err_limited[MSB_LIM:0]};
                           end
                    5'd17: begin // integration sum with over/under flow check
                            {extra, Yi} <= {Ys[MSB_FP], Ys} + {temp[MSB_FP], temp};
                           end
                    5'd16: begin
                            ovrflow <=  ({extra, Yi[MSB_FP]} == 2'b01 );
                            undrflow <=  ({extra, Yi[MSB_FP]} == 2'b10 );
                           end
                    5'd15: Ys <= (ovrflow)? (~(1<<MSB_FP)): 
					            (undrflow)?    (1<<MSB_FP): Yi;
                    5'd14: begin // integration saturation, Ys is output
                           if(Ys[MSB_FP])
                            Ys <= (Ys < int_lim_neg) ? int_lim_neg : Ys;  // Q.0
                           else
                            Ys <= (Ys > integ_lim) ? integ_lim : Ys;      // Q.0
                           end	
                    // ******* cascade lead/lag with unity gain *******						  			  				  
                    5'd13: begin
                            Y1 <= Ys + (bs * y_old >>> fsze);   // Q.0
                            Y2 <= as * err_old >>> fsze;        // Q.0
                           end
                    5'd12: begin
                            result1 <= (Y1 - Y2);          // Q.0
							err_old <= Ys;                 // Q.0
                           end
                    5'd11: y_old <= result1;               // Q.0
                    5'd10: ; // empty
				    // ******* apply gains to result *******	
					5'd09: begin
                            result1 <= (result1 * ks);     // Q.fsze
                           end					
					5'd08: begin
                            result1 <= (result1 * ki) >>> (2*fsze + 3);	 // Q.0
                            end				
                    // ******* apply offset, may be able to remove this? *******					  					  
                    5'd7: begin
                          y_out = result1 - offset;            // Q.0
                          end
                    5'd6: begin
                          y_off = $signed(y_out + 19'd1027);   // positive offset reduces this value
                          end
                    5'd5: begin
                          outp2 <= (y_off > $signed(19'd2047))? (19'd2047) : (y_off < $signed(16'd048)) ? 16'd048 : y_off ;                                         
                          end
                    5'd4: ; // empty state        
                    5'd3: sample_1 <= 1'b1;         // data ready
                    5'd2: ; // empty state          // do nothing here
                    5'd1: sample_1 <= 1'b0;         // reset sample
					5'd0: ; // empty state
                    default: ;    // don't do anything
                endcase
            end  // state machine
        end // computation
    end
// ----------------------------------------------------------------------------------
// debug
/*
     ila_0 lead_lag_dbg (
    .clk(sys_clk),              // 1, input wire clk

    .trig_in(result1),          // input wire trig_in
    .trig_in_ack(trig_in_ack),  // output wire trig_in_ack
    .probe0(err),               // 22
    .probe1(err_limited),       // 18
    .probe2(Y1),                // 29
    .probe3(Y2),                // 29
    .probe4(Y3),                // 29
    .probe5(result1),           // 19
    .probe6(counter),           // 4
    .probe7(err_lim),           // 18
    .probe8(err_lim_neg)        // 18
);
*/
endmodule

