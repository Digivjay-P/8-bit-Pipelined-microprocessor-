`timescale 1ns / 1ps


module MUX_2x1_wb(
    input  [7:0] wrtdata0,  // write data from normal pipeline
    input  [2:0] destreg0,  // dest reg from normal pipeline
    input        write0,    // write enable from normal pipeline
    input  [7:0] wrtdata1,  // write data from Matrix_Output
    input  [2:0] destreg1,  // dest reg from Matrix_Output
    input        write1,    // write enable from Matrix_Output
    input        sel,       // 0 = pipeline, 1 = matrix output
    output reg [7:0] wrtdata_out,
    output reg [2:0] destreg_out,
    output reg       write_out
);

always @(*) begin
    if (sel == 1'b1) begin
        wrtdata_out = wrtdata1;
        destreg_out = destreg1;
        write_out   = write1;
    end else begin
        wrtdata_out = wrtdata0;
        destreg_out = destreg0;
        write_out   = write0;
    end
end

endmodule

