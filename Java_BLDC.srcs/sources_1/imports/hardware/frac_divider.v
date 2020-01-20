`include "timescale.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: Tremaine Consulting Group
// Engineer:    Brian Tremaine
//
// Create Date:    13:25:19 04/27/2016
// Design Name:
// Module Name:    frac_divider
// Project Name:
// Target Devices: Spartan 3AN XC3S50ATQ144
// Tool versions:
// Description:    fractional divider. Divides by N+1 for m cycles and divided
//                 by N for Mbar-m cycles. Average divide is Nbar = N + m/Mbar
//         In this code Mbar is parameterized using fsze
//
//                 Output q is high for count < (N>>1) otherwise q is low.
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module frac_divider #(parameter WIDTH = 20, FSZE = 3)(
    // inputs
    input sys_clk,
    input sync_rst_n,
    input [31:0] N,                 // integer divider
    input signed [WIDTH-1:0] mf,    // fractional divide ...yyyyy.xxx
    // outputs
    output [WIDTH-1:0] count,
    output q_out
    );

localparam MSB = WIDTH-1;
localparam fsze = FSZE;             // # of fractional bits

reg [WIDTH-1:0] ncoarse;
wire [fsze-1:0] nfine;
reg signed [WIDTH-1:0] mfs4;
reg [WIDTH-1:0] M;
reg [fsze-1:0] mcnt;

wire rst;

// initialize
initial begin
    mcnt[fsze-1:0] = 0;
    end

// assign
    assign q_out = (M > (N>>1)) ? 1'b1 : 1'b0;
    assign count = M;
    assign rst = ~sync_rst_n;
    assign nfine = mf[fsze-1:0] ;

////// combinatorial /////
always @ (*) begin
      mfs4 <= mf>>>fsze;
      ncoarse <= (mf[MSB]) ? (N - (~mfs4+1)) : (N + mfs4);
end

// --------------------------------------------------

    always @(negedge sys_clk, posedge rst) begin        // mcnt register
        if(rst)
            mcnt <= 0;
        else begin
                if(M==0) begin
                    if(mcnt!=0)
                        mcnt <= mcnt-1;
                    else
                        mcnt <= (1 << fsze) - 1;
                end
        end
    end

    always @(negedge sys_clk, posedge rst) begin        // M register
        if (rst)
            M <= N-1;
        else begin
                if(M !=0)
                    M <= M - 1'b1;
                else if( mcnt < nfine)
                    M <= ncoarse + 1'b1;
                else
                    M <= ncoarse;
        end
    end

endmodule
