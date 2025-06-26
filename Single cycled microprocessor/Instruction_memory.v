`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: OpenAI (modified by request)
// Module Name: Instruction_memory
//////////////////////////////////////////////////////////////////////////////////

module Instruction_memory (
    input [7:0] address,
    output [15:0] instruction
);
    reg [15:0] mem[0:511];

initial begin
    mem[0] = 16'h1205; // LI R0, 5
mem[1] = 16'h140A; // LI R1, 10
mem[2] = 16'h2650; // ADD R2 = R1 + R0
mem[3] = 16'h9610; // STORE R2 -> [R1] (i.e., mem[10])
mem[4] = 16'hD810; // LOAD R3 <- [R1] (i.e., mem[10])
mem[5] = 16'h0000; // Dummy NOP

end


    assign instruction = (address < 9) ? mem[address] : 16'h0000;
endmodule
