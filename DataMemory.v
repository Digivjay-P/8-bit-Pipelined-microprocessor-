`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.06.2025 16:27:04
// Design Name: 
// Module Name: DataMemory
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


module DataMemory(
input CLK,  
input MemWrite, 
input [7:0] ALUResult, 
input [7:0] WriteData, 
output reg [7:0] ReadData
  );
  
  reg [7:0] Memory [255:0];
  integer i;
  
initial begin
ReadData <= 0;
for ( i=0; i<256; i = i + 1)
begin 
Memory[i] = i; 
end 
end 

always @ (posedge CLK) 
begin
if (MemWrite == 1'b1)
Memory[ALUResult] <= WriteData;
else 
ReadData <= Memory[ALUResult];
end

endmodule
