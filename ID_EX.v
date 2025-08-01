`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:54:03
// Design Name: 
// Module Name: ID_EX
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

//ID and EX
module ID_EX(
  input [7:0] SrcA, SrcB_reg, pcplus1, ImmExt,
  input [2:0] destreg, Rs1D, Rs2D,
  input RegWrite, ALUSrc, ResultSrc, MemWrite, is_matrix_mult_d,
  input [3:0] ALUControl,

  output reg [7:0] SrcA_out, SrcB_reg_out, pcplus1_out, ImmExt_out,
  output reg [2:0] destreg_out, Rs1E, Rs2E,
  output reg RegWrite_out, ALUSrc_out, ResultSrc_out, MemWrite_out,is_matrix_mult_e,
  output reg [3:0] ALUControl_out,

  input clk, input reset, FlushE
);
  always @(posedge clk or posedge reset) begin
    if (reset || FlushE) begin
      SrcA_out <= 0; SrcB_reg_out <= 0; pcplus1_out <= 0; ImmExt_out <= 0;
      destreg_out <= 0; Rs1E <= 0; Rs2E <= 0; RegWrite_out <= 0; ALUSrc_out <= 0; ResultSrc_out <= 0;
      MemWrite_out <= 0; ALUControl_out <= 0; is_matrix_mult_e <= 0;
    end else begin
      SrcA_out <= SrcA; SrcB_reg_out <= SrcB_reg; pcplus1_out <= pcplus1; ImmExt_out <= ImmExt;
      destreg_out <= destreg; Rs1E <= Rs1D; Rs2E <= Rs2D;
      RegWrite_out <= RegWrite; ALUSrc_out <= ALUSrc; ResultSrc_out <= ResultSrc;
      MemWrite_out <= MemWrite; ALUControl_out <= ALUControl; is_matrix_mult_e <= is_matrix_mult_d;
    end
  end
endmodule
