`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.06.2025 00:26:04
// Design Name: 
// Module Name: Control_Unit
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




module ControlUnit(
    input [3:0] opcode,
    input [2:0] funct,
    input Zero,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemWrite,
    output reg [3:0] ALUControl,
    output reg ResultSrc,
    output reg PCSrc
);

parameter OP_RTYPE        = 4'b0000;
parameter OP_LOAD_IM      = 4'b0001;
parameter OP_LOAD         = 4'b0010;
parameter OP_STORE        = 4'b0011;
parameter OP_JUMP         = 4'b0100;
parameter OP_EQUAL_TO     = 4'b0101;
parameter OP_RIGHT_SHIFT  = 4'b0110;
parameter OP_LEFT_SHIFT   = 4'b0111;

parameter F_ADD           = 3'b000;
parameter F_SUBTRACT      = 3'b001;
parameter F_MULTIPLY      = 3'b010;
parameter F_DIVIDE        = 3'b011;
parameter F_AND           = 3'b100;
parameter F_OR            = 3'b101;
parameter F_XOR           = 3'b110;
parameter F_NOT           = 3'b111;

parameter ALU_ADD         = 4'b1000;
parameter ALU_SUBTRACT    = 4'b1001;
parameter ALU_MULTIPLY    = 4'b1010;
parameter ALU_DIVIDE      = 4'b1011;
parameter ALU_AND         = 4'b1100;
parameter ALU_OR          = 4'b1101;
parameter ALU_NOT         = 4'b1110;
parameter ALU_XOR         = 4'b1111;
parameter ALU_RIGHT_SHIFT = 4'b0110;
parameter ALU_LEFT_SHIFT  = 4'b0111;
parameter ALU_EQUAL_TO    = 4'b0101;

always @(*) begin
    RegWrite    = 0;
    ALUSrc      = 0;
    MemWrite    = 0;
    ALUControl  = 4'b0000;
    ResultSrc   = 0;
    PCSrc       = 0;

    case (opcode)
        OP_RTYPE: begin
            RegWrite = 1;
            ALUSrc   = 0;
            case (funct)
                F_ADD:      ALUControl = ALU_ADD;
                F_SUBTRACT: ALUControl = ALU_SUBTRACT;
                F_MULTIPLY: ALUControl = ALU_MULTIPLY;
                F_DIVIDE:   ALUControl = ALU_DIVIDE;
                F_AND:      ALUControl = ALU_AND;
                F_OR:       ALUControl = ALU_OR;
                F_XOR:      ALUControl = ALU_XOR;
                F_NOT:      ALUControl = ALU_NOT;
               
            endcase
        end

        OP_LOAD_IM: begin
            RegWrite   = 1;
            ALUSrc     = 1;
            ALUControl = ALU_ADD;
            ResultSrc  = 0;
        end

        OP_LOAD: begin
            RegWrite   = 1;
            ALUSrc     = 1;
            ALUControl = ALU_ADD;
            ResultSrc  = 1;
        end

        OP_STORE: begin
            ALUSrc     = 1;
            MemWrite   = 1;
            ALUControl = ALU_ADD;
        end

        OP_EQUAL_TO: begin
            ALUControl = ALU_EQUAL_TO;
            PCSrc      = Zero;
        end

        OP_JUMP: begin
            PCSrc = 1;
        end

        OP_RIGHT_SHIFT: begin
            RegWrite   = 1;
            ALUSrc     = 0;
            ALUControl = ALU_RIGHT_SHIFT;
        end

        OP_LEFT_SHIFT: begin
            RegWrite   = 1;
            ALUSrc     = 0;
            ALUControl = ALU_LEFT_SHIFT;
        end
        default: ALUControl = 4'b0000;
    endcase
         
end

endmodule