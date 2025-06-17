`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.06.2025 21:36:49
// Design Name: 
// Module Name: DataMemory_tb
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


module DataMemory_tb;
reg CLK;
reg MemWrite;
reg [7:0] ALUResult;
reg [7:0] WriteData;
wire [7:0] ReadData;

DataMemory a1( CLK, MemWrite, ALUResult, WriteData, ReadData);


initial begin 
CLK = 0;
MemWrite = 0;
WriteData = 8'b10001111;
ALUResult = 7;

$monitor($time, " ALUResult = %d, WriteData = %d, ReadData = %d", ALUResult, WriteData, ReadData);
#20 
MemWrite = 1;
#20
MemWrite = 0;
WriteData = 8'hAD;
ALUResult = 10;
#20
MemWrite = 1;
#20
ALUResult = 7;
#20
$finish();
end 
always #10  CLK = ~CLK;
endmodule
