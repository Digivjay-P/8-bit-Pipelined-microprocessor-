`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 16:23:36
// Design Name: 
// Module Name: Extender
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

//EXTENDER
module Extender(
    input  [2:0] in0,
    output reg [7:0] ImmExt
);
    always @(*) begin
        if (in0[2] == 0)
            ImmExt = {5'b00000, in0}; // positive
        else
            ImmExt = {5'b11111, in0}; // negative
    end
endmodule
