`timescale 1ns / 1ps
//IF and ID
module IF_ID(
  input [7:0] pcplus1_f,
  input [15:0] instr_f,
  input [7:0] pc_f,
  output reg [7:0] pc_d,
  output reg [7:0] pcplus1_d,
  output reg [15:0] instr_d,
  output reg is_matrix_mult_d,
  input is_matrix_mult_f,
  input clk, input reset,
  input StallD, FlushD
);

  always @(posedge clk or posedge reset) begin
    if (reset || FlushD) begin
      pcplus1_d <= 8'd0;
      instr_d <= 16'd0;
      pc_d <= 8'd0;
      is_matrix_mult_d <= 1'd0;
    end else if (!StallD) begin
      pcplus1_d <= pcplus1_f;
      instr_d <= instr_f;
      pc_d <= pc_f;
      is_matrix_mult_d <= is_matrix_mult_f;
    end
  end
endmodule
