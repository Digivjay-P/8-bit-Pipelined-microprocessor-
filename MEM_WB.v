`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:56:26
// Design Name: 
// Module Name: MEM_WB
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

//MEM and WB
module MEM_WB(
  input [7:0] ReadData, ALUResult, pcplus1,
  input [2:0] destreg,
  input RegWrite, ResultSrc,
  output reg [7:0] ReadData_out, ALUResult_out, pcplus1_out,
  output reg [2:0] destreg_out,
  output reg RegWrite_out, ResultSrc_out,
  input clk, input reset
);

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      ReadData_out <= 0; ALUResult_out <= 0; pcplus1_out <= 0;
      destreg_out <= 0; RegWrite_out <= 0; ResultSrc_out <= 0;
    end else begin
      ReadData_out <= ReadData; ALUResult_out <= ALUResult; pcplus1_out <= pcplus1;
      destreg_out <= destreg;
      RegWrite_out <= RegWrite; ResultSrc_out <= ResultSrc;
    end
  end
endmodule
