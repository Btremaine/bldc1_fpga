//-----------------------------------------------------------------------------
//  Project  : Laser ALT FPGA
//  Module   : rst_gen.v

//  Parent   : laser_top.v 

//  Children : sync_1ck.v
//  Description: 

//     This module is the reset generator for the design.
//     It takes the asynchronous reset in (from the IBUF), and generates
//     a synchronous reset in the clock domain.
//
//  Parameters:
//     None
//
//  Notes: 

`include "..\include\timescale.v"
`include "..\include\defines.v"

//-----------------------------------------------------------------------
module rst_gen(
           // Inputs
           input reset_n,        // reset from fpga pin
           input clk,
           // Outputs
           output sync_rst_n     // synchronized reset         
) ;

// Change to positive polarity.
wire reset = ~reset_n;
wire sync_rst ;

// Synchronize reset to clock domain.
sync_1ck sync_rst1 (
   // Inputs
   .clk                (clk),
   .rst_n              (1'b1),
   .async_in           (reset),
   // Outputs
   .sync_out           (sync_rst)
);

// Falling edge asynchronous, rising edge synchronous.
assign sync_rst_n    = ~(sync_rst || reset);

endmodule
