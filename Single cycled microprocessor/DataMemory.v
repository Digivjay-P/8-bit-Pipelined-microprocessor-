`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: 
// 
// Description: Updated to expose internal memory location for testing
//////////////////////////////////////////////////////////////////////////////////

module DataMemory(
    input CLK,  
    input MemWrite, 
    input [7:0] ALUResult, 
    input [7:0] WriteData, 
    output reg [7:0] ReadData,

    // ? Add this line to expose Memory[10]
    output [7:0] memory_10
);

    reg [7:0] Memory [255:0];
    integer i;
    
    // Initialize memory
    initial begin
        ReadData <= 0;
        for (i = 0; i < 256; i = i + 1)
            Memory[i] = i; 
    end 

    // Read/Write logic
    always @ (posedge CLK) begin
        if (MemWrite)
            Memory[ALUResult] <= WriteData;
        else
            ReadData <= Memory[ALUResult];
    end

    // ? Assign internal memory location to output wire
    assign memory_10 = Memory[10];

endmodule
