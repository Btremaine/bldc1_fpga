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

`include "..\include\timescale.v"
`include "..\include\defines.v"

//-----------------------------------------------------------------------
module debounce #(parameter DELAY=100)(
           // Inputs
           input sync_rst_n,     // synchronized reset
           input clk,
		   input inpt,
           // Outputs
           output outp           // synchronized reset         
) ;

reg inpt_sync ;
reg [12:0] count;			// DELAY cannot exceed this reg width

// synchronize input
always @(posedge clk)
    if( ~sync_rst_n)
		inpt_sync <= 0 ;
	else
		inpt_sync <= inpt ;
		
// debounce
always @(posedge clk)
	if(~sync_rst_n)
		count <= 0 ;
	else begin
		if( inpt_sync & (count < DELAY))
			count <= count + 1;
		if(~inpt_sync & (count > 0))
			count <= count - 1;		
	end
	
assign outp = (count > (DELAY>>>1))? 1 : 0;


endmodule
