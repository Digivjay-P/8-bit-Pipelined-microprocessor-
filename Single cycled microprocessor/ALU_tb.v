`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.06.2025 11:02:26
// Design Name: 
// Module Name: ALU_tb
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


module ALU_tb;
reg [7:0]SrcA, SrcB;
reg [3:0] ALUControl;
wire [7:0] ALUResult;
wire Zero;
integer i;
ALU a1(SrcA, SrcB, ALUControl, ALUResult, Zero);


initial 
begin
SrcA = 8'b10;
SrcB = 8'b100;
ALUControl = 4'b0;

$monitor($time, " SrcA = %b, SrcB = %b, ALUControl = %b, ALUResult = %b", SrcA, SrcB, ALUControl, ALUResult);

for (i=0; i <=12; i = i + 1)
begin 
ALUControl = ALUControl + 1;
#10;
end
$finish;
end 

endmodule
