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
    initial
    begin
    $readmemh("program.hex", mem);
    end 
    assign instruction = mem[address];
endmodule
