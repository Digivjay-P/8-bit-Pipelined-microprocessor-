`timescale 1ns / 1ps
module Register_file(rdata1, rdata2, wrtData, srcreg1, srcreg2, destreg, write, clk);
input clk, write;
input [2:0] srcreg1, srcreg2, destreg;
input [7:0] wrtData;
output [7:0] rdata1, rdata2;

reg [7:0] regfile [0:7];

assign rdata1 = regfile[srcreg1];
assign rdata2 = regfile[srcreg2];

always @(posedge clk) begin
  if (write)
    regfile[destreg] <= wrtData;
end

endmodule