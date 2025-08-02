`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 18:57:50
// Design Name: 
// Module Name: Hazard_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.02 - Added stall on matrix writeback operation
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Hazard_unit( 
    input [2:0] Rs1D, Rs2D, Rs1E, Rs2E, a1_d, a1_e, a1_w, a1_m,
    input [3:0] opcode,
    input regwrite_m, regwrite_w, ResultSrc_e, pcsrc,
    input matrix_write_in_progress,             // Added input to stall pipeline during matrix writeback
    output reg StallF, StallD, FlushD, FlushE,
    output reg [1:0] ForwardAE, ForwardBE
);

wire hzd;
assign hzd = (ResultSrc_e && (a1_e != 0) &&
              ((a1_e == a1_d) || (a1_e == Rs1D) || (a1_e == Rs2D)) &&
              (opcode != 4'b0101) && (opcode != 4'b0100));

always @(*) begin
    StallF  = 0;
    StallD  = 0;
    FlushE  = 0;
    FlushD  = 0;

    if (hzd) begin
        StallF = 1;
        StallD = 1;
        FlushE = 1;
    end

    if (pcsrc)
        FlushD = 1;

    // Stall pipeline during matrix writeback operation
    if (matrix_write_in_progress) begin
        StallF = 1;
        StallD = 1;
        // Optionally you can flush or stall E stage as well if needed
    end
end
                          
always @(*) begin
    // ForwardAE (for SrcA_e)
    if (regwrite_m && (a1_m != 0) && (a1_m == Rs1E))
        ForwardAE = 2'b01; // Forward from EX/MEM
    else if (regwrite_w && (a1_w != 0) && (a1_w == Rs1E))
        ForwardAE = 2'b10; // Forward from MEM/WB
    else
        ForwardAE = 2'b00; // No forwarding

    // ForwardBE (for SrcB)
    if (regwrite_m && (a1_m != 0) && (a1_m == Rs2E))
        ForwardBE = 2'b01; // Forward from EX/MEM
    else if (regwrite_w && (a1_w != 0) && (a1_w == Rs2E))
        ForwardBE = 2'b10; // Forward from MEM/WB
    else
        ForwardBE = 2'b00; // No forwarding
end

endmodule
