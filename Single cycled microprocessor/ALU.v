`timescale 1ns / 1ps
module ALU(
    input  [7:0] SrcA, 
    input  [7:0] SrcB,
    input  [3:0] ALUControl,
    output reg [7:0] ALUResult,
    output reg Zero
);

parameter ADD          = 4'b1000;
parameter SUBTRACT     = 4'b1001;
parameter MULTIPLY     = 4'b1010;
parameter DIVIDE       = 4'b1011;
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
    case (ALUControl)
        ADD:          ALUResult = SrcA + SrcB;
        SUBTRACT:     ALUResult = SrcA + (~SrcB + 1'b1);
        MULTIPLY:     ALUResult = SrcA * SrcB;
        DIVIDE:       ALUResult = (SrcB != 0) ? SrcA / SrcB : 8'b0;
        AND:          ALUResult = SrcA & SrcB;
        OR:           ALUResult = SrcA | SrcB;
        NOT:          ALUResult = ~SrcA;
        XOR:          ALUResult = SrcA ^ SrcB;
        Right_Shift:  ALUResult = $signed(SrcA) >>> 1;
        Left_Shift:   ALUResult = $signed(SrcA) << 1;
        Equal_to:     ALUResult = (SrcA == SrcB) ? 8'b00000001 : 8'b00000000;
        default:      ALUResult = 8'b00000000;
    endcase
    Zero = (ALUResult == 0) ? 1'b1 : 1'b0;
end

endmodule