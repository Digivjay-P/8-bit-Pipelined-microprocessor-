`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 16:20:14
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

//DATA MEMORY
module DataMemory(
input CLK,  
input reset,
input MemWrite, 
input [7:0] ALUResult, 
input [7:0] WriteData, 
output reg [7:0] ReadData
  );
  
  reg [7:0] Memory [255:0];
  integer i;
always @ (posedge CLK or posedge reset) begin  
if (reset) begin
ReadData <= 0;
for ( i=0; i<256; i = i + 1) 
  Memory[i] = 0; 

end 
else begin
if (MemWrite == 1'b1)
Memory[ALUResult] <= WriteData;
end
  assign ReadData = Memory[ALUResult];
end 
endmodule
