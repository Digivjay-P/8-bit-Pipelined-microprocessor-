`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.06.2025 00:29:18
// Design Name: 
// Module Name: tb_reg
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

module tb_Register_file;

    // Inputs
    reg clk;
    reg reset;
    reg write;
    reg [2:0] srcreg1;
    reg [2:0] srcreg2;
    reg [2:0] destreg;
    reg [7:0] wrtData;

    // Outputs
    wire [7:0] rdata1;
    wire [7:0] rdata2;

    // Instantiate the Unit Under Test (UUT)
    Register_file uut (
        .rdata1(rdata1),
        .rdata2(rdata2),
        .wrtData(wrtData),
        .srcreg1(srcreg1),
        .srcreg2(srcreg2),
        .destreg(destreg),
        .write(write),
        .reset(reset),
        .clk(clk)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        $display("Starting Register File Testbench...");
        clk = 0;
        reset = 1;
        write = 0;
        srcreg1 = 3'b000;
        srcreg2 = 3'b001;
        destreg = 3'b000;
        wrtData = 8'd0;

        // Hold reset for a few clock cycles
        #10;
        reset = 0;

        // Write 42 to register 1
        #10;
        write = 1;
        destreg = 3'b001;
        wrtData = 8'd42;

        // Write 99 to register 2
        #10;
        destreg = 3'b010;
        wrtData = 8'd99;

        // Disable write
        #10;
        write = 0;

        // Read from reg 1 and reg 2
        srcreg1 = 3'b001;
        srcreg2 = 3'b010;
        #10;

        // Display output
        $display("rdata1 (should be 42): %d", rdata1);
        $display("rdata2 (should be 99): %d", rdata2);

        // Check reset again
        #10;
        reset = 1;
        #10;
        reset = 0;

        // Read from reg 1 and 2 again (should be 0)
        #10;
        $display("After reset, rdata1: %d (expected 0)", rdata1);
        $display("After reset, rdata2: %d (expected 0)", rdata2);

        $display("Testbench finished.");
        $finish;
    end

endmodule
