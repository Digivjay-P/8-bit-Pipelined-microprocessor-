`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.06.2025 18:58:13
// Design Name: 
// Module Name: Instruction_memory
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


module Instruction_memory (address, instruction

    );
    input [7:0] address;
    output [15:0] instruction;
    reg [15:0] mem[0:511];
    initial begin
    mem[0] = 16'h1125; 
    mem[1] = 16'h114A;
    mem[2] = 16'h0632; 
    mem[3] = 16'h1289; 
    mem[4] = 16'h2912; 
    mem[5] = 16'h2D12; 
    mem[6] = 16'h2F92; 
    mem[7] = 16'h500A; 
    mem[8] = 16'h4000; 
end
    
    assign instruction = mem[address];
endmodule
