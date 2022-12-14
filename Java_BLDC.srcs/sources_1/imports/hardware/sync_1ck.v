
`include "timescale.v"

module sync_1ck
#(parameter WIDTH=1)
(
   // Inputs
   input clk,                         // Clock synchronizing data to
   input rst,
   input [WIDTH-1:0] async_in,
   // Outputs
   output [WIDTH-1:0] sync_out       // Synchronous data out
);

reg[WIDTH-1:0]         data_q = {WIDTH{1'b0}};       // First stage synchronization register
reg[WIDTH-1:0]         data_2q = {WIDTH{1'b0}};      // Second stage synchronization register

always @(posedge clk or posedge rst) begin
   if (rst) begin
      data_q   <= {WIDTH{1'b0}};
      data_2q  <= {WIDTH{1'b0}};
   end else begin
      data_q   <= async_in;
      data_2q  <= data_q;
   end
end

assign
   sync_out = data_2q;

endmodule
