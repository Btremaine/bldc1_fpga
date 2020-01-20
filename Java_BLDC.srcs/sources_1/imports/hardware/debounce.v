//-----------------------------------------------------------------------------
//  Project  : BLDC Motor control
//  Module   : debounce.v

//  Parent   : Top_Frac_pwm.v

//  Children : none
//  Description:

//     This module debounces an external signal from the BLDC control chip
//
//
//  Parameters:
//     None
//
//  Notes:

`include "timescale.v"

//-----------------------------------------------------------------------
module debounce #(parameter DELAY=100)(
           // Inputs
           input sync_rst_n,     // synchronized reset
           input clk,
           input inpt,
           // Outputs
           output reg outp,
           output dbl
) ;

reg  [2:0] inpt_sync ;
reg [12:0] count;            // DELAY cannot exceed this reg width

// synchronize input
always @(posedge clk)
    if(~sync_rst_n) begin
        inpt_sync <= 0;
    end else begin
        inpt_sync <= {inpt_sync[1:0], inpt} ;        // avoid metastable state // replace 3 Flip-Flops with a 3 bit shift register, result is the same, code is smaller
    end

wire input_event = (inpt_sync[1] != inpt_sync[2]);

// if input didn't change for DELAY cycles considering it stable
wire stable = (count == DELAY);

// count register
always @(posedge clk)
    if(~sync_rst_n)
        count <= 0 ;
    else if (input_event)
        count <= 0;
    else if (!stable)
        count <= count + 1;

// outp register
always @(posedge clk)
    if(~sync_rst_n)
        outp <= 0 ;
    else if (stable)
        outp <= inpt_sync[2];

assign dbl = 0;

endmodule
