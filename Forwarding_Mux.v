`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:43:21
// Design Name: 
// Module Name: Forwarding_Mux
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

module Forwarding_Mux( Src_in, Result, ALU_Result, forward, Src_out);
input [7:0] Src_in;
input [7:0] Result;
input [7:0] ALU_Result;
input [1:0] forward;
output [7:0] Src_out;

assign Src_out = (forward == 2'b00) ? Src_in:
                 (forward == 2'b01) ? ALU_Result:
                 (forward == 2'b10) ? Result:
                 Src_in;
                 endmodule 