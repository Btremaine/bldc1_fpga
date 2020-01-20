`include "timescale.v"
//////////////////////////////////////////////////////////////////////////////////
// Company:     Prysm Inc.
// Engineer:    Brian Tremaine
//
// Create Date:    01/08/2019
// Design Name:
// Module Name:    PI_comp
// Project Name:
// Target Devices:  Spartan 3AN XC3S50ATQ144
// Tool versions:
// Description:     digital PI compensation for Java BLDC
//                  Uses signed fixed point arithmetic
//                  User gains {ki,kp,ko} are in registers.
//                  Calculations of difference eqt. are pipelined.
//
//                  'err_limited' is phase error with +/- saturation limit applied.
//                  'Yi' is trial integration before integral saturation limit applied.
//                  'Ys' is integral after saturation limit applied
//
//                   y1(k) = Kp * u(k) + Ki * LIM(Ys(k-1) + u(k))
//                   y(k)  = Ko * y1(k)
//
//                   output low-pass filter (klpf)
//                   yout(k) = c * yout(k-1) + (1 - c) * y(k-1)
// Dependencies:
//
// Revision:   11/6/2019 - fix low-pass filter
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module pi_comp #(parameter WIDTH_U= 24, WIDTH_ERR=22, WIDTH_LIM=16,
                 WIDTH_FP=30, fsze = 6) (
    // input
    input sys_clk,                              // system clock
    input rst,                                  // active high reset
    input signed [WIDTH_ERR-1:0] err,           // signed error
    input [WIDTH_LIM-1:0] dlim,                 // err phase limit value
    input [WIDTH_FP-1:0] intlim,                // integrator limit
    input [15:0] ki,                            // Q.fsze   0 < x < 1
    input [15:0] kp,                            // Q.fsze   0 < x < 1
    input [15:0] klpf,                          // Q.fsze   0 < x < 1
    input [15:0] k0,                            // Q.fsze   0 < x < 1
    input enable,
    input process_in,
    // outputs
    output signed [WIDTH_U-1:0] uk
    );

localparam MSB_ERR = WIDTH_ERR-1;               // msb of err input
localparam MSB_LIM = WIDTH_LIM-1;               // msb of limited phase error
localparam MSB_FP = WIDTH_FP-1;                 // msb of fixed point
localparam MSB_U = WIDTH_U-1;                   // msb of output

reg signed  [WIDTH_FP-1:0] result1;             // temporary register
reg signed  [WIDTH_FP-1:0] result2;             // temporary register
reg signed  [WIDTH_FP-1:0] y_out;               // temporary register
reg signed  [WIDTH_FP-1:0] y_old;               // temporary register
reg signed  [WIDTH_FP-1:0] Yi;                  // integrator raw
reg signed  [WIDTH_FP-1:0] Ys;                  // integrator saturated             //
reg signed  [WIDTH_FP-1:0] R1 ;                 // intermediate result
reg signed  [WIDTH_FP-1:0] Yo ;                 // intermediate result
reg signed  [WIDTH_FP-1:0] Yo_old ;             // intermediate result
reg signed  [WIDTH_FP-1:0] temp;                // temporary register

reg process_dly;
reg [3:0] counter;
reg undrflow;
reg ovrflow;
reg extra;
reg [WIDTH_LIM-1 :0] err_lim;                   // unsigned limit on error
reg [WIDTH_FP-1:0] integ_lim;                   // unsigned limit on integrator
reg signed [WIDTH_LIM-1:0] err_lim_neg;
reg signed [WIDTH_LIM-1:0] err_limited;         // limited phase error
reg signed [WIDTH_FP-1:0] int_lim_neg;          //


wire signed [fsze:0] kis;                       // use signed ops
wire signed [fsze:0] kps;
wire signed [fsze+3:0] k0s;
wire signed [fsze+3:0] cs;
wire signed [WIDTH_LIM-1:0] err_lim_sgnd;
wire signed [WIDTH_FP-1:0] int_lim_sgnd;

// assign
assign uk = y_out;  // final output value
assign kis = ki;
assign kps = kp;
assign k0s = k0;
assign cs = klpf;
assign err_lim_sgnd = err_lim;
assign int_lim_sgnd = integ_lim;

//
    always @ (posedge sys_clk) begin
            err_lim_neg <= ~err_lim_sgnd + 1;
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
                result2 <= 0;
                y_out <= 0;
                y_old <= 0;
                Yi <= 0;
                Ys <= 0;
                R1 <= 0;
                Yo <= 0;
				Yo_old <= 0;
                temp <= 0;
                err_limited <= 0;
                extra <= 0;
                ovrflow <= 0;
                undrflow <= 0;
                counter <= 0;
        end
        else begin // computation
                if (process_in & !process_dly)
                    counter <= 4'd13;
                else if (counter !=0)
                    counter <= counter-1;
                begin // state machine  
                // compensation pipeline
                case (counter)
                    4'd13: begin // phase error limiting
                          if(err[MSB_ERR])  // phase limit err (signed)
                            err_limited <= (err < err_lim_neg) ? err_lim_neg : err; // Q.0
                          else
                            err_limited <= (err > err_lim) ? err_lim : err;
                          end
                          // temp == extend err_limited to width WIDTH_FP
                    4'd12: temp <= {{(WIDTH_FP-WIDTH_LIM){err_limited[MSB_LIM]}}, err_limited[MSB_LIM:0]};
                    4'd11: begin // integration over/under flow check
                            {extra, Yi} <= {Ys[MSB_FP], Ys} + {temp[MSB_FP], temp};
                          end
                    4'd10: begin
                            ovrflow <=  ({extra, Yi[MSB_FP]} == 2'b01 );
                            undrflow <=  ({extra, Yi[MSB_FP]} == 2'b10 );
                          end
                    4'd9: Ys <= (ovrflow)? ~(1<<MSB_FP): (undrflow)? 1<<MSB_FP: Yi;
                    4'd8: begin // integration saturation
                          if(Ys[MSB_FP])
                            Ys <= (Ys < int_lim_neg) ? int_lim_neg : Ys;  // Q.0
                          else
                            Ys <= (Ys > integ_lim) ? integ_lim : Ys;      // Q.0
                          end
                          // coefficient computation
                    4'd7: R1 <= err_limited * kps + Ys * kis;   // Q.fsze, preserve Ys
                    4'd6: result1 <= R1 * k0s >>> fsze;         // Q.(2*fsze) -> Q.fsze
                    4'd5: result2 <= result1 >>> fsze;
                    4'd4: y_old <= result2;
                    4'd3: // output lpf
					      // yout(k) = c * yout(k-1) + (1 - c) * y(k-1)
                          Yo <= cs * Yo_old + ((1<<<fsze) - cs) * y_old ; // Q.fsze
                    4'd2: begin
                          y_out <= Yo >>> fsze ;  // Q.fsze -> Q.0
                          end
                    4'd1: begin
                          Yo_old <= y_out;
                          end
					4'd0: ; // do nothing
                    default: ;    // don't do anything
                endcase
                end  // state machine
        end  // computation
    end
// ----------------------------------------------------------------------------------
// debug
/*
ila_1 PI_comp_dbg (
    .clk(sys_clk), // input wire clk

    .trig_in(counter),     // input wire trig_in
    .trig_in_ack(trig_in_ack),// output wire trig_in_ack
    .probe0(err),         // 22
    .probe1(err_limited), // 16
    .probe2(Yi),          // 30
    .probe3(Ys),          // 30
    .probe4(R1),          // 30
    .probe5(temp),        // 30
    .probe6(result1),     // 30
    .probe7(counter),     // 4
    .probe8(ovrflow),     // 1
    .probe9(undrflow),    // 1
    .probe10(int_lim_neg), // 30
    .probe11(integ_lim),   // 30
    .probe12(kis),         // 7
    .probe13(kps),         // 7
    .probe14(result2),     // 24
    .probe15(result3)      // 24
);
*/


endmodule
