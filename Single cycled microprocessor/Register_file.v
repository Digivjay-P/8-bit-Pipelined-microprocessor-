`timescale 1ns / 1ps

module Register_file(
    output [7:0] rdata1,
    output [7:0] rdata2,
    input [7:0] wrtData,
    input [2:0] srcreg1,
    input [2:0] srcreg2,
    input [2:0] destreg,
    input write,
    input clk
);

    reg [7:0] regfile [0:7];

    // Initialize all registers to 0
    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1)
            regfile[i] = 8'd0;
    end

    // Read ports
    assign rdata1 = regfile[srcreg1];
    assign rdata2 = regfile[srcreg2];

    // Write logic
    always @(posedge clk) begin
        if (write)
            regfile[destreg] <= wrtData;
    end

    // Expose selected registers to testbench
    wire [7:0] rf_test_1 = regfile[1];
    wire [7:0] rf_test_2 = regfile[2];
    wire [7:0] rf_test_3 = regfile[3];
    wire [7:0] rf_test_4 = regfile[4];

endmodule
