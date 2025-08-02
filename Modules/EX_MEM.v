`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:55:01
// Design Name: 
// Module Name: EX_MEM
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

//EX and MEM
module EX_MEM(
  input [7:0] ALUResult, SrcB_reg, pcplus1,
  input [2:0] destreg,
  input RegWrite, MemWrite, ResultSrc, is_matrix_mult_e,

  output reg [7:0] ALUResult_out, WriteData_out, pcplus1_out,
  output reg [2:0] destreg_out,
  output reg RegWrite_out, MemWrite_out, ResultSrc_out, is_matrix_mult_m,

  input clk, input reset
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      ALUResult_out <= 0; WriteData_out <= 0; pcplus1_out <= 0;
      destreg_out <= 0; RegWrite_out <= 0; MemWrite_out <= 0; ResultSrc_out <= 0; 
      is_matrix_mult_m <= 0;
    end else begin
      ALUResult_out <= ALUResult; WriteData_out <= SrcB_reg; pcplus1_out <= pcplus1;
      destreg_out <= destreg;
      RegWrite_out <= RegWrite; MemWrite_out <= MemWrite; ResultSrc_out <= ResultSrc;
      is_matrix_mult_m <= is_matrix_mult_e;
    end
  end
endmodule
