`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.06.2025 00:18:11
// Design Name: 
// Module Name: Single_Cycle_Processor
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


module Single_Cycle_Processor(

input CLK,
output [7:0] result 
   );
wire pcsrc;
wire [7:0] pcnext;
wire [7:0] pc;
wire [15:0] instr;
wire [7:0] SrcA;
wire [7:0] SrcB;
wire Zero;
wire [7:0] ALUResult;
wire [3:0] ALUControl;
wire ALUSrc, ResultSrc;
wire [7:0] pcplus1;
wire [7:0] pctarget;
wire [2:0] a1, a2, a3;
wire [3:0] opcode;
wire [2:0] funct;
wire [7:0] SrcB_reg;
wire [7:0] wrtdata;
wire [7:0] read_data;
wire MemWrite;
wire regwrite;
wire [7:0] ImmExt; //To extender

assign pcplus1 = pc + a1;
assign pctarget = pc + ImmExt;
//assign pcnext = pcsrc ? pctarget : pcplus1;


pc p0(.pcnext(pcnext), .pc(pc), .clk(CLK));
MUX_2x1 start(.in0(pcplus1), .in1(pctarget), .select(pcsrc), .out0(pcnext));

Instruction_memory i0(.address(pc), .instruction(instr));
assign opcode = instr[15:12];
assign a1 = instr[8:6];  //Destination register 
assign funct = instr[11:9];
assign a2 = instr[5:3];
assign a3 = instr[2:0];
Extender e0(.in0(a3), .ImmExt(ImmExt));
ControlUnit c0(.opcode(opcode), .funct(funct), .Zero(Zero), .ALUSrc(ALUSrc), .MemWrite(MemWrite), .ALUControl(ALUControl), .ResultSrc(ResultSrc), .PCSrc(pcsrc), .RegWrite(regwrite));
Register_file r0(.clk(CLK), .write(regwrite), .destreg(a1), .srcreg1(a2), .srcreg2(a3), .rdata1(SrcA), .rdata2(SrcB_reg), .wrtData(wrtdata));
MUX_2x1 alu(.in0(SrcB_reg), .in1(ImmExt), .select(ALUSrc), .out0(SrcB));
ALU a0( .SrcA(SrcA), .SrcB(SrcB), .Zero(Zero), .ALUResult(ALUResult), .ALUControl(ALUControl));
DataMemory d0(.CLK(CLK), .MemWrite(MemWrite), .ALUResult(ALUResult), .WriteData(SrcB_reg), .ReadData(read_data));
MUX_2x1 read(.in0(ALUResult), .in1(read_data), .select(ResultSrc), .out0(wrtdata));
assign result = wrtdata;


endmodule
