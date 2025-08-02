`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.07.2025 16:21:16
// Design Name: 
// Module Name: ALU
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

//ALU
module ALU(
    input  [7:0] SrcA, 
    input  [7:0] SrcB,
    input  [3:0] ALUControl,
    output reg [7:0] ALUResult,
    output reg Zero
);

reg [7:0] ALUResult_int;

parameter ADD          = 4'b1000;
parameter SUBTRACT     = 4'b1001;
parameter MULTIPLY     = 4'b1010;
parameter MATRIX_MULT  = 4'b1011;
parameter AND          = 4'b1100;
parameter OR           = 4'b1101;
parameter NOT          = 4'b1110;
parameter XOR          = 4'b1111;
parameter Right_Shift  = 4'b0110;
parameter Left_Shift   = 4'b0111;
parameter LOAD         = 4'b0010;  
parameter STORE        = 4'b0011;   
parameter JUMP         = 4'b0100;
parameter Equal_to     = 4'b0101;

always @(*) begin
    $display("ALU: SrcA=%d, SrcB=%d, ALUControl=%b, ALUResult=%d", SrcA, SrcB, ALUControl, ALUResult);
    // Existing case statement

    case (ALUControl)
        ADD:          ALUResult_int = SrcA + SrcB;
        SUBTRACT:     ALUResult_int = SrcA + (~SrcB + 1'b1);
        MULTIPLY:     ALUResult_int = SrcA * SrcB;
        MATRIX_MULT:  begin
        ALUResult_int = 8'b00000000;
end
        AND:          ALUResult_int = SrcA & SrcB;
        OR:           ALUResult_int = SrcA | SrcB;
        NOT:          ALUResult_int = ~SrcA;
        XOR:          ALUResult_int = SrcA ^ SrcB;
        Right_Shift:  ALUResult_int = $signed(SrcA) >>> 1;
        Left_Shift:   ALUResult_int = $signed(SrcA) << 1;
        Equal_to:     ALUResult_int = (SrcA == SrcB) ? 8'b00000001 : 8'b00000000;
        default:   begin
          ALUResult_int = 8'b00000000;
    
        end
    endcase
      ALUResult = ALUResult_int;
      Zero = (ALUResult_int == 0) ? 1'b1 : 1'b0;
end


endmodule

