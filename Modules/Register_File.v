`timescale 1ns / 1ps
//REGISTER 

module Register_file(
    rdata1, 
    rdata2, 
    wrtData, 
    srcreg1, 
    srcreg2, 
    destreg, 
    write, 
    clk, 
    reset,
    A,
    B,
    ReadDataA0, ReadDataA1, ReadDataA2, ReadDataA3,
    ReadDataB0, ReadDataB1, ReadDataB2, ReadDataB3
);

output [31:0] A,B;
input clk, reset, write;
input [2:0] srcreg1, srcreg2, destreg;
input [7:0] wrtData;
output [7:0] rdata1, rdata2;
output [7:0] ReadDataA0, ReadDataA1, ReadDataA2, ReadDataA3;
output [7:0] ReadDataB0, ReadDataB1, ReadDataB2, ReadDataB3; 
integer i;
reg [7:0] regfile [0:7];

assign rdata1 = regfile[srcreg1];
assign rdata2 = regfile[srcreg2];

assign ReadDataA0 = regfile[0];
assign ReadDataA1 = regfile[1];
assign ReadDataA2 = regfile[2];
assign ReadDataA3 = regfile[3];

assign A = {ReadDataA0, ReadDataA1, ReadDataA2, ReadDataA3};

assign ReadDataB0 = regfile[4];
assign ReadDataB1 = regfile[5];
assign ReadDataB2 = regfile[6];
assign ReadDataB3 = regfile[7];

assign B = {ReadDataB0, ReadDataB1, ReadDataB2, ReadDataB3};

always @ (posedge   clk or posedge reset )begin
if (reset) begin
  for ( i = 0; i < 8; i = i + 1)begin
    regfile[i] <= i;
    end    
end

else  if (write) begin 
    regfile[destreg] <= wrtData;
end
end

endmodule
