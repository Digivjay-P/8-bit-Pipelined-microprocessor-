`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.06.2025 23:58:01
// Design Name: 
// Module Name: Register_file
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


module Register_file(rdata1, rdata2, wrtData, srcreg1, srcreg2, destreg, write, reset, clk);
input clk,write,reset;
input [2:0] srcreg1, srcreg2, destreg;
input [7:0] wrtData ;
output [7:0] rdata1, rdata2;
integer k;

reg [7:0] regfile [0:7];

assign rdata1 = regfile[srcreg1];
assign rdata2 = regfile[srcreg2];

always @(posedge clk)
  begin 
   if(reset) begin
     for(k=0;k<8;k=k+1)begin
       regfile[k]<=0;
     end
   end  
   else begin
      if(write) 
        regfile[destreg] <= wrtData;
   end 
end     
endmodule
