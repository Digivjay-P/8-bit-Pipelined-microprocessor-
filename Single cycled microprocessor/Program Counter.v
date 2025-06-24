`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.06.2025 10:39:01
// Design Name: 
// Module Name: Register_file
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

module pc(pc, clk, pcnext);
input clk;
input [7:0] pcnext;
output reg [7:0] pc;
initial begin
pc = 8'b0;
end
always @(posedge clk)
begin
pc <= pcnext;
end

    
endmodule