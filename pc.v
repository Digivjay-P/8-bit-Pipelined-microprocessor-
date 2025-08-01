`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 16:30:03
// Design Name: 
// Module Name: pc
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//Program Counter
module pc(pc, clk, reset, pcnext, StallF);
input clk, reset;
input [7:0] pcnext;
input StallF;
output reg [7:0] pc;
always  @(posedge   clk or posedge reset) begin

if(reset)
pc <= 8'b0;
else if (!StallF)
pc <= pcnext;
end
endmodule

