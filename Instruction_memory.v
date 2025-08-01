`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:44:32
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

//INSTRUCTION MEMORY
module Instruction_memory (address, instruction

    );
   input [7:0] address;
    output  reg [15:0] instruction;
   reg [15:0] mem[0:511];
integer i;
    initial begin
      mem[0] = 16'h1044; // LOAD_IM R1, 4
      mem[1] = 16'h1081; // LOAD R2, 1

      mem[2] = 16'h00ca; // ADD     R3, R1, R2   (funct=000)
      mem[3] = 16'h0319; // SUB     R4, R3, R1   (funct=001)
      mem[4] = 16'h098a; // AND     R6, R1, R2   (funct=100)
      mem[5] = 16'h0daa; // OR      R6, R5, R2   (funct=101)
      mem[6] = 16'h0dca; // XOR     R7, R1, R2   (funct=110)
      mem[7] = 16'h0f48; // NOT     R5, R1, X    (funct=111, ignore R2)

      mem[8] = 16'h058a; // MULT    R6, R1, R2   (funct=010)
      mem[9] = 16'h072e; // MATRIX MULTIPLICATION (funct=011)
      //mem[9] = 16'h07ca; // DIV     R7, R1, R2   (funct=011)

      mem[10] = 16'h7108; // LEFT_SHIFT  R4, R1, X (funct via opcode = 0111)
      mem[11] = 16'h6108; // RIGHT_SHIFT R4, R1, X (opcode = 0110)

      mem[12] = 16'h5280; // EQUAL R1==R2? ? branch (opcode = 0101)
      mem[13] = 16'h4002; // JUMP (opcode = 0100)
      



     for ( i = 14; i < 512; i = i + 1)
    mem[i] = 16'h0000;

    end
 always @(*) begin
     instruction = mem[address];
     end
endmodule