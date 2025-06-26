`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.06.2025 21:40:31
// Design Name: 
// Module Name: MUX 2x1
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


module MUX_2x1(
input [7:0] in0, input [7:0] in1, input select, output [7:0] out0
    );
    
    assign out0 = (select ==0) ? in0 : in1;
endmodule
