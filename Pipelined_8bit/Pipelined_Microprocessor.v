
`timescale 1ns / 1ps
//DATA MEMORY
module DataMemory(
input CLK,  
input reset,
input MemWrite, 
input [7:0] ALUResult, 
input [7:0] WriteData, 
output reg [7:0] ReadData
  );
  
  reg [7:0] Memory [255:0];
  integer i;
always @ (posedge CLK or posedge reset) begin  
if (reset) begin
ReadData <= 0;
for ( i=0; i<256; i = i + 1) 
  Memory[i] = 0; 

end 
else begin
if (MemWrite == 1'b1)
Memory[ALUResult] <= WriteData;
else 
ReadData <= Memory[ALUResult];
end
end 
endmodule

//ALU
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
    $display("ALU: SrcA=%d, SrcB=%d, ALUControl=%b, ALUResult=%d", SrcA, SrcB, ALUControl, ALUResult);
    // Existing case statement

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
        Equal_to:    ALUResult = (SrcA == SrcB) ? 8'b00000001 : 8'b00000000;
        default:   begin
          ALUResult = 8'b00000000;
    
        end
    endcase
      Zero = (ALUResult == 0) ? 1'b1 : 1'b0;
end

endmodule
 
//CONTROL UNIT

module ControlUnit(
    input [3:0] opcode,
    input [2:0] funct,
    input Zero,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemWrite,
    output reg [3:0] ALUControl,
    output reg ResultSrc,
    output reg PCSrc
);

parameter OP_RTYPE        = 4'b0000;
parameter OP_LOAD_IM      = 4'b0001;
parameter OP_LOAD         = 4'b0010;
parameter OP_STORE        = 4'b0011;
parameter OP_JUMP         = 4'b0100;
parameter OP_EQUAL_TO     = 4'b0101;
parameter OP_RIGHT_SHIFT  = 4'b0110;
parameter OP_LEFT_SHIFT   = 4'b0111;

parameter F_ADD           = 3'b000;
parameter F_SUBTRACT      = 3'b001;
parameter F_MULTIPLY      = 3'b010;
parameter F_DIVIDE        = 3'b011;
parameter F_AND           = 3'b100;
parameter F_OR            = 3'b101;
parameter F_XOR           = 3'b110;
parameter F_NOT           = 3'b111;

parameter ALU_ADD         = 4'b1000;
parameter ALU_SUBTRACT    = 4'b1001;
parameter ALU_MULTIPLY    = 4'b1010;
parameter ALU_DIVIDE      = 4'b1011;
parameter ALU_AND         = 4'b1100;
parameter ALU_OR          = 4'b1101;
parameter ALU_NOT         = 4'b1110;
parameter ALU_XOR         = 4'b1111;
parameter ALU_RIGHT_SHIFT = 4'b0110;
parameter ALU_LEFT_SHIFT  = 4'b0111;
parameter ALU_EQUAL_TO    = 4'b0101;

  always @(*) begin
    $display("ControlUnit: opcode=%b, funct=%b, RegWrite=%b, ALUControl=%b", opcode, funct, RegWrite, ALUControl);
    // Existing code

    RegWrite    = 0;
    ALUSrc      = 0;
    MemWrite    = 0;
    ALUControl  = 4'b0000;
    ResultSrc   = 0;
    PCSrc       = 0;

    case (opcode)
        OP_RTYPE: begin
            RegWrite = 1;
            ALUSrc   = 0;
            case (funct)
                F_ADD:      ALUControl = ALU_ADD;
                F_SUBTRACT: ALUControl = ALU_SUBTRACT;
                F_MULTIPLY: ALUControl = ALU_MULTIPLY;
                F_DIVIDE:   ALUControl = ALU_DIVIDE;
                F_AND:      ALUControl = ALU_AND;
                F_OR:       ALUControl = ALU_OR;
                F_XOR:      ALUControl = ALU_XOR;
                F_NOT:      ALUControl = ALU_NOT;
               
            endcase
        end

        OP_LOAD_IM: begin
            RegWrite   = 1;
            ALUSrc     = 1;
            ALUControl = ALU_ADD;
            ResultSrc  = 0;
        end

        OP_LOAD: begin
            RegWrite   = 1;
            ALUSrc     = 1;
            ALUControl = ALU_ADD;
            ResultSrc  = 1;
        end

        OP_STORE: begin
            ALUSrc     = 1;
            MemWrite   = 1;
            ALUControl = ALU_ADD;
        end

        OP_EQUAL_TO: begin
            ALUControl = ALU_EQUAL_TO;
            PCSrc      = Zero;
        end

        OP_JUMP: begin
            PCSrc = 1;
        end

        OP_RIGHT_SHIFT: begin
            RegWrite   = 1;
            ALUSrc     = 0;
            ALUControl = ALU_RIGHT_SHIFT;
        end

        OP_LEFT_SHIFT: begin
            RegWrite   = 1;
            ALUSrc     = 0;
            ALUControl = ALU_LEFT_SHIFT;
        end
        default: ALUControl = 4'b0000;
    endcase
         
end

endmodule

//EXTENDER
module Extender(
    input  [2:0] in0,
    output reg [7:0] ImmExt
);
    always @(*) begin
        if (in0[2] == 0)
            ImmExt = {5'b00000, in0}; // positive
        else
            ImmExt = {5'b11111, in0}; // negative
    end
endmodule
//REGISTER 

module Register_file(rdata1, rdata2, wrtData, srcreg1, srcreg2, destreg, write, clk, reset);
input clk, reset, write;
input [2:0] srcreg1, srcreg2, destreg;
input [7:0] wrtData;
output [7:0] rdata1, rdata2;
integer i;
reg [7:0] regfile [0:7];

assign rdata1 = regfile[srcreg1];
assign rdata2 = regfile[srcreg2];
always @ (posedge   clk or posedge reset )begin
if (reset) begin
  for ( i = 0; i < 8; i = i + 1)
    regfile[i] = i;
end
else  if (write) begin 
    regfile[destreg] <= wrtData;
end
end
endmodule

//MUX 2x1
module MUX_2x1(
input [7:0] in0, input [7:0] in1, input select, output [7:0] out0
    );
    
    assign out0 = (select ==0) ? in0 : in1;
endmodule

//Program Counter
module pc(pc, clk, reset, pcnext);
input clk, reset;
input [7:0] pcnext;
output reg [7:0] pc;
always  @(posedge   clk or posedge reset) begin

if(reset)
pc = 8'b0;
else
pc <= pcnext;
end

    
endmodule


//INSTRUCTION MEMORY
module Instruction_memory (address, instruction

    );
   input [7:0] address;
    output  reg [15:0] instruction;
   reg [15:0] mem[0:511];
integer i;
    initial begin
      mem[0] = 16'h1044; // LOAD_IM R1, 4
      mem[1] = 16'h1081; // LOAD R2, 1

      mem[2] = 16'h00ca; // ADD     R3, R1, R2   (funct=000)
      mem[3] = 16'h0319; // SUB     R4, R3, R1   (funct=001)
      mem[4] = 16'h098a; // AND     R6, R1, R2   (funct=100)
      mem[5] = 16'h0daa; // OR      R6, R5, R2   (funct=101)
      mem[6] = 16'h0dca; // XOR     R7, R1, R2   (funct=110)
      mem[7] = 16'h0f48; // NOT     R5, R1, X    (funct=111, ignore R2)

      mem[8] = 16'h058a; // MULT    R6, R1, R2   (funct=010)
      mem[9] = 16'h07ca; // DIV     R7, R1, R2   (funct=011)

      mem[10] = 16'h7108; // LEFT_SHIFT  R4, R1, X (funct via opcode = 0111)
      mem[11] = 16'h6108; // RIGHT_SHIFT R4, R1, X (opcode = 0110)

      mem[12] = 16'h5280; // EQUAL R1==R2? ? branch (opcode = 0101)
      mem[13] = 16'h4002; // JUMP (opcode = 0100)



     for ( i = 14; i < 512; i = i + 1)
    mem[i] = 16'h0000;

    end
 always @(*) begin
     instruction = mem[address];
     end
endmodule

//PROCESSOR 

module Single_Cycle_Processor(

input CLK,
input reset,
output [7:0] result 
   );
wire pcsrc;
wire [7:0] pcnext;
wire [7:0] pc;
wire [15:0] instr;
wire signed [7:0] SrcA;
wire signed [7:0] SrcB;
wire Zero;
wire [7:0] ALUResult;
wire [3:0] ALUControl;
wire ALUSrc, ResultSrc;
wire [7:0] pcplus1;
wire [7:0] pctarget;
wire [2:0] a1, a2, a3;
wire [3:0] opcode;
wire [2:0] funct;
wire [7:0] SrcB_reg;
wire [7:0] wrtdata;
wire [7:0] read_data;
wire MemWrite;
wire regwrite;
wire [7:0] ImmExt; //To extender




pc p0(.pcnext(pcnext),. reset(reset), .pc(pc), .clk(CLK));


Instruction_memory i0(.address(pc), .instruction(instr));
assign opcode = instr[15:12];
assign pcnext = (opcode == 4'b0101 && Zero) ? (pc + ImmExt) : 
  (pcsrc) ? ImmExt :
                (pc + 1);


assign a1 = instr[8:6];  //Destination register 
assign funct = instr[11:9];
assign a2 = instr[5:3];
assign a3 = instr[2:0];
Extender e0(.in0(a3), .ImmExt(ImmExt));
ControlUnit c0(.opcode(opcode), .funct(funct), .Zero(Zero), .ALUSrc(ALUSrc), .MemWrite(MemWrite), .ALUControl(ALUControl), .ResultSrc(ResultSrc), .PCSrc(pcsrc), .RegWrite(regwrite));
Register_file r0(.clk(CLK), .reset(reset), .write(regwrite), .destreg(a1), .srcreg1(a2), .srcreg2(a3), .rdata1(SrcA), .rdata2(SrcB_reg), .wrtData(wrtdata));
MUX_2x1 alu(.in0(SrcB_reg), .in1(ImmExt), .select(ALUSrc), .out0(SrcB));
ALU a0( .SrcA(SrcA), .SrcB(SrcB), .Zero(Zero), .ALUResult(ALUResult), .ALUControl(ALUControl));
DataMemory d0(.CLK(CLK), .reset(reset), .MemWrite(MemWrite), .ALUResult(ALUResult), .WriteData(SrcB_reg), .ReadData(read_data));
MUX_2x1 read(.in0(ALUResult), .in1(read_data), .select(ResultSrc), .out0(wrtdata));
assign result = wrtdata;


endmodule



/*module ALU_Instructions();

    // Inputs
    reg CLK;

    // Outputs
    wire [7:0] result;

    // Instantiate the Unit Under Test (UUT)
    Single_Cycle_Processor uut (
        .CLK(CLK),
        .result(result)
    );

    // Clock generation
    initial begin
        CLK = 0;
        forever #5 CLK = ~CLK; // 10ns clock period
    end

    // Simulation control
    initial begin
        // Optionally initialize memories here if needed
        $display("Simulation Start");
      $dumpfile("wave.vcd");
      $dumpvars(0, ALU_Instructions);
        
        // Run simulation for some cycles
        #100;

        // Display final result
        $display("Final Result: %d", result);
        $finish;
        
    end

endmodule */
/*
module tb_instruction_memory();
  reg [7:0] address;
  wire [15:0] instruction;
  Instruction_memory i1(.address(address), .instruction(instruction));
  initial begin 
    $display("Simulation initiated");
    $dumpfile("wave.vcd");
    $dumpvars(0, ALU_Instructions);
   
    for (integer i=0; i<10; i = i + 1) begin
      address = i;
      $display("Address: %0d, Instruction: %h", address, instruction);
        end

        $finish;
    end

endmodule
*/

module ALU_Instructions;
    reg CLK;
    reg reset;
    wire [7:0] result;
    Single_Cycle_Processor uut (
        .CLK(CLK), 
        .reset(reset),
        .result(result)
    );
     initial begin
        reset = 1;
        #3;
        reset = 0;
    end
  initial begin
    uut.r0.regfile[1] = 8'h05; // Set R1=5
    uut.r0.regfile[2] = 8'h06; // Set R2=6
end
    initial begin
        CLK = 0;
        forever #5 CLK = ~CLK; 
    end
integer i;
    initial begin
        $display("Simulation Start");
      $dumpfile("wave.vcd");
      $dumpvars(0, ALU_Instructions);
      #10;
      
      for (  i = 0; i <=13 ; i = i+1) begin
        @(posedge CLK);
        @(posedge CLK);
        #1;
       $monitor("Time=%0t PC=%0d Instr=%h ALUResult=%d WriteData=%d RegWrite=%b MemWrite=%b",
    $time, uut.pc, uut.instr, uut.ALUResult, uut.wrtdata, uut.regwrite, uut.MemWrite);
 
        #1;
      end
      #100;
      $display("Simulation end..");
        $finish;
        
    end
endmodule



//Latches
//IF and ID
module IF_ID(
  input [7:0] pcplus1,
  input [15:0] instr,
  output reg [7:0] pcplus1_out,
  output reg [15:0] instr_out,
  input clk, input reset
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      pcplus1_out <= 8'd0;
      instr_out <= 16'd0;
    end else begin
      pcplus1_out <= pcplus1;
      instr_out <= instr;
    end
  end
endmodule



//ID and EX
module ID_EX(
  input [7:0] SrcA, SrcB_reg, pcplus1, ImmExt,
  input [2:0] destreg,
  input RegWrite, ALUSrc, ResultSrc, MemWrite,
  input [3:0] ALUControl,

  output reg [7:0] SrcA_out, SrcB_reg_out, pcplus1_out, ImmExt_out,
  output reg [2:0] destreg_out,
  output reg RegWrite_out, ALUSrc_out, ResultSrc_out, MemWrite_out,
  output reg [3:0] ALUControl_out,

  input clk, input reset
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      SrcA_out <= 0; SrcB_reg_out <= 0; pcplus1_out <= 0; ImmExt_out <= 0;
      destreg_out <= 0; RegWrite_out <= 0; ALUSrc_out <= 0; ResultSrc_out <= 0;
      MemWrite_out <= 0; ALUControl_out <= 0;
    end else begin
      SrcA_out <= SrcA; SrcB_reg_out <= SrcB_reg; pcplus1_out <= pcplus1; ImmExt_out <= ImmExt;
      destreg_out <= destreg;
      RegWrite_out <= RegWrite; ALUSrc_out <= ALUSrc; ResultSrc_out <= ResultSrc;
      MemWrite_out <= MemWrite; ALUControl_out <= ALUControl;
    end
  end
endmodule



//EX and MEM
module EX_MEM(
  input [7:0] ALUResult, SrcB_reg, pcplus1,
  input [2:0] destreg,
  input RegWrite, MemWrite, ResultSrc,

  output reg [7:0] ALUResult_out, WriteData_out, pcplus1_out,
  output reg [2:0] destreg_out,
  output reg RegWrite_out, MemWrite_out, ResultSrc_out,

  input clk, input reset
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      ALUResult_out <= 0; WriteData_out <= 0; pcplus1_out <= 0;
      destreg_out <= 0; RegWrite_out <= 0; MemWrite_out <= 0; ResultSrc_out <= 0;
    end else begin
      ALUResult_out <= ALUResult; WriteData_out <= SrcB_reg; pcplus1_out <= pcplus1;
      destreg_out <= destreg;
      RegWrite_out <= RegWrite; MemWrite_out <= MemWrite; ResultSrc_out <= ResultSrc;
    end
  end
endmodule



//MEM and WB
module EX_MEM(
  input [7:0] ALUResult, SrcB_reg, pcplus1,
  input [2:0] destreg,
  input RegWrite, MemWrite, ResultSrc,

  output reg [7:0] ALUResult_out, WriteData_out, pcplus1_out,
  output reg [2:0] destreg_out,
  output reg RegWrite_out, MemWrite_out, ResultSrc_out,

  input clk, input reset
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      ALUResult_out <= 0; WriteData_out <= 0; pcplus1_out <= 0;
      destreg_out <= 0; RegWrite_out <= 0; MemWrite_out <= 0; ResultSrc_out <= 0;
    end else begin
      ALUResult_out <= ALUResult; WriteData_out <= SrcB_reg; pcplus1_out <= pcplus1;
      destreg_out <= destreg;
      RegWrite_out <= RegWrite; MemWrite_out <= MemWrite; ResultSrc_out <= ResultSrc;
    end
  end
endmodule
