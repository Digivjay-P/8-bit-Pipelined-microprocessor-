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

    // This block handles synchronous writes on the clock edge.
    always @ (posedge CLK or posedge reset) begin  
        if (reset) begin
            // On reset, clear the memory.
            for ( i=0; i<256; i = i + 1) 
                Memory[i] <= 0; 
        end else begin
            // On a normal clock edge, write to memory if MemWrite is high.
            if (MemWrite == 1'b1)
                Memory[ALUResult] <= WriteData;
        end
  assign ReadData = Memory[ALUResult];
    
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

    //-- MODIFIED: Added SLT parameter
    parameter SLT           = 4'b0000; 
    parameter ADD           = 4'b1000;
    parameter SUBTRACT      = 4'b1001;
    parameter MULTIPLY      = 4'b1010;
    parameter DIVIDE        = 4'b1011;
    parameter AND           = 4'b1100;
    parameter OR            = 4'b1101;
    parameter NOT           = 4'b1110;
    parameter XOR           = 4'b1111;
    parameter Right_Shift   = 4'b0110;
    parameter Left_Shift    = 4'b0111;
    parameter LOAD          = 4'b0010;  
    parameter STORE         = 4'b0011;   
    parameter JUMP          = 4'b0100;
    parameter Equal_to      = 4'b0101;

    always @(*) begin
        //-- MODIFIED: This $display was commented out as it can clutter simulation logs.
        // $display("ALU: SrcA=%d, SrcB=%d, ALUControl=%b, ALUResult=%d", SrcA, SrcB, ALUControl, ALUResult);

        case (ALUControl)
            //-- MODIFIED: Added SLT case
            SLT:         ALUResult = ($signed(SrcA) < $signed(SrcB)) ? 8'd1 : 8'd0;
            ADD:         ALUResult = SrcA + SrcB;
            SUBTRACT:    ALUResult = SrcA + (~SrcB + 1'b1);
            MULTIPLY:    ALUResult = SrcA * SrcB;
            DIVIDE:      ALUResult = (SrcB != 0) ? SrcA / SrcB : 8'b0;
            AND:         ALUResult = SrcA & SrcB;
            OR:          ALUResult = SrcA | SrcB;
            NOT:         ALUResult = ~SrcA;
            XOR:         ALUResult = SrcA ^ SrcB;
            Right_Shift: ALUResult = $signed(SrcA) >>> 1;
            Left_Shift:  ALUResult = $signed(SrcA) << 1;
            Equal_to:    ALUResult = (SrcA == SrcB) ? 8'b00000001 : 8'b00000000;
            default:     ALUResult = 8'b00000000;
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
    parameter OP_ADDI         = 4'b1000;
    parameter OP_BEQ          = 4'b1001;

    parameter F_ADD           = 3'b000;
    parameter F_SUBTRACT      = 3'b001;
    parameter F_MULTIPLY      = 3'b010;
    parameter F_DIVIDE        = 3'b011;
    parameter F_AND           = 3'b100;
    parameter F_OR            = 3'b101;
    parameter F_XOR           = 3'b110;
    parameter F_NOT           = 3'b111;
    //-- MODIFIED: Added SLT funct code
    parameter F_SLT           = 3'b111; 

    //-- MODIFIED: Added SLT ALU operation code
    parameter ALU_SLT         = 4'b0000;
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
        //-- MODIFIED: This $display was commented out as it can clutter simulation logs.
        // $display("ControlUnit: opcode=%b, funct=%b, RegWrite=%b, ALUControl=%b", opcode, funct, RegWrite, ALUControl);

        RegWrite   = 0;
        ALUSrc     = 0;
        MemWrite   = 0;
        ALUControl = 4'b0000;
        ResultSrc  = 0;
        PCSrc      = 0;

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
                    //-- MODIFIED: Added SLT case. Note that F_NOT and F_SLT have the same code (3'b111).
                    // This means you cannot have a separate NOT instruction with this funct code.
                    F_SLT:      ALUControl = ALU_SLT; 
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
                ResultSrc  = 0;
            end

            OP_STORE: begin
                RegWrite = 1; // This is a bug from your original code. STORE should not write to a register.
                ALUSrc     = 1;
                ResultSrc = 0;
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
            
            OP_ADDI: begin
                RegWrite   = 1;
                ALUSrc     = 1;
                ALUControl = ALU_ADD;
                ResultSrc  = 0;
            end
            
            OP_BEQ: begin
                RegWrite = 0;
                ALUControl = ALU_EQUAL_TO;
                MemWrite = 0;
                ResultSrc = 0;
                ALUSrc = 0;
                PCSrc = Zero;
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
    
    always @ (posedge clk or posedge reset )begin
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
    input [7:0] in0, 
    input [7:0] in1, 
    input select, 
    output [7:0] out0
);
    assign out0 = (select ==0) ? in0 : in1;
endmodule

module Forwarding_Mux( Src_in, Result, ALU_Result, forward, Src_out);
    input [7:0] Src_in;
    input [7:0] Result;
    input [7:0] ALU_Result;
    input [1:0] forward;
    output [7:0] Src_out;

    assign Src_out = (forward == 2'b00) ? Src_in:
                     (forward == 2'b01) ? ALU_Result:
                     (forward == 2'b10) ? Result:
                     Src_in;
endmodule 

//Program Counter
module pc(pc, clk, reset, pcnext, stallF);
    input clk, reset;
    input [7:0] pcnext;
    input stallF;
    output reg [7:0] pc;
    
    always @(posedge clk or posedge reset) begin
        if(reset)
            pc <= 8'b0;
        else if (!stallF)
            pc <= pcnext;
    end
endmodule


//INSTRUCTION MEMORY
module Instruction_memory (
    input [7:0] address,
    output  reg [15:0] instruction
);
    reg [15:0] mem[0:511];
    integer i;
    
    initial begin
        // Initialization
        mem[0] = 16'h8045;  // ADDI R1 = RO+5
        mem[1] = 16'h8080;  // ADDI R3 = R0+0
        mem[2] = 16'h3050;  // R1=R2
        mem[3] = 16'h8081;  // ADDI R1 = R0+2
        
        // Main Program (Hazard-prone, using R0-R7 only)
        mem[4] = 16'h3050; //Load R1 <-  R2
        mem[5] = 16'h8042; // ADDI R1 <- R0+2
        mem[6] = 16'h8082; // ADDI R2 <- R0+2
        mem[7] = 16'h3050; // R1=R2
        mem[8] = 16'h80C3; // R3=R0+3
        mem[9] = 16'h8040; // R1 = R0+0
        mem[10] = 16'h0399; // SUB R6 = R3-R1
        mem[11] = 16'h9C07; // BEQ R6 = R0
        mem[12] = 16'h039a; // SUB R6=R3-R2
        mem[13] = 16'h9C07; // Branch R6=R0
        mem[14] = 16'h2308; // LOAD R4=R1
        mem[15] = 16'h2350; // LOAD R5=R2
        mem[16] = 16'h0fac; // SLT R6  R5 R4
        mem[17] = 16'h4010; // Jump
        for ( i = 18; i < 512; i = i + 1)
            mem[i] = 16'h0000;
    end
    
    always @(*) begin
        instruction = mem[address];
    end
endmodule


//Latches
//IF and ID
module IF_ID(
    input [7:0] pcplus1_f,
    input [15:0] instr_f,
    input [7:0] pc_f,
    output reg [7:0] pc_d,
    output reg [7:0] pcplus1_d,
    output reg [15:0] instr_d,
    input clk, input reset,
    input stallD, flushD
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pcplus1_d <= 8'd0;
            instr_d <= 16'd0;
            pc_d <= 8'd0;
        end else if (flushD) begin
            pcplus1_d <= 8'd0;
            instr_d <= 16'd0;
            pc_d <= 8'd0;
        end else if (!stallD) begin
            pcplus1_d <= pcplus1_f;
            instr_d <= instr_f;
            pc_d <= pc_f;
        end
    end
endmodule

//ID and EX
module ID_EX(
    input [7:0] SrcA, SrcB_reg, pcplus1, ImmExt,
    input [2:0] destreg, Rs1D, Rs2D,
    input RegWrite, ALUSrc, ResultSrc, MemWrite,
    input [3:0] ALUControl,
    output reg [7:0] SrcA_out, SrcB_reg_out, pcplus1_out, ImmExt_out,
    output reg [2:0] destreg_out, Rs1E, Rs2E,
    output reg RegWrite_out, ALUSrc_out, ResultSrc_out, MemWrite_out,
    output reg [3:0] ALUControl_out,
    input clk, input reset, flushE
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            SrcA_out <= 0; SrcB_reg_out <= 0; pcplus1_out <= 0; ImmExt_out <= 0;
            destreg_out <= 0; Rs1E <= 0; Rs2E <= 0; RegWrite_out <= 0; ALUSrc_out <= 0; ResultSrc_out <= 0;
            MemWrite_out <= 0; ALUControl_out <= 0;
        end else begin
            if (flushE) begin
                SrcA_out <= 0; SrcB_reg_out <= 0; pcplus1_out <= 0; ImmExt_out <= 0;
                destreg_out <= 0; Rs1E <= 0; Rs2E <= 0; RegWrite_out <= 0; ALUSrc_out <= 0; ResultSrc_out <= 0;
                MemWrite_out <= 0; ALUControl_out <= 0;
            end else begin
                SrcA_out <= SrcA; SrcB_reg_out <= SrcB_reg; pcplus1_out <= pcplus1; ImmExt_out <= ImmExt;
                destreg_out <= destreg; Rs1E <= Rs1D; Rs2E <= Rs2D;
                RegWrite_out <= RegWrite; ALUSrc_out <= ALUSrc; ResultSrc_out <= ResultSrc;
                MemWrite_out <= MemWrite; ALUControl_out <= ALUControl;
            end
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
module MEM_WB(
    input [7:0] ReadData, ALUResult, pcplus1,
    input [2:0] destreg,
    input RegWrite, ResultSrc,
    output reg [7:0] ReadData_out, ALUResult_out, pcplus1_out,
    output reg [2:0] destreg_out,
    output reg RegWrite_out, ResultSrc_out,
    input clk, input reset
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ReadData_out <= 0; ALUResult_out <= 0; pcplus1_out <= 0;
            destreg_out <= 0; RegWrite_out <= 0; ResultSrc_out <= 0;
        end else begin
            ReadData_out <= ReadData; ALUResult_out <= ALUResult; pcplus1_out <= pcplus1;
            destreg_out <= destreg;
            RegWrite_out <= RegWrite; ResultSrc_out <= ResultSrc;
        end
    end
endmodule

module Hazard_unit( 
    input [2:0] Rs1D, Rs2D, Rs1E, Rs2E, a1_d, a1_e, a1_w, a1_m,
    input [3:0] opcode,
    input regwrite_m, regwrite_w, ResultSrc_e, pcsrc,
    output reg StallF, StallD, FlushD, FlushE,
    output reg [1:0] ForwardAE, ForwardBE
);
    wire hzd;

    assign hzd = (ResultSrc_e && (a1_e != 0) &&
                  ((a1_e == a1_d) || (a1_e == Rs1D) || (a1_e == Rs2D)) &&
                  (opcode != 4'b0101) && (opcode != 4'b0100));
                  
    always @(*) begin
        StallF = hzd;
        StallD = hzd;
        FlushE = hzd;
        FlushD = pcsrc; // Flush for branches/jumps
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


//PIPELINED PROCESSOR 
module Pipelined_Processor(
    input CLK,
    input reset,
    output [7:0] result 
);
    wire pcsrc;
    wire [7:0] pcnext_f;
    wire [7:0] pc_f, pc_d;
    wire [15:0] instr_f, instr_d;
    wire signed [7:0] SrcA_d, SrcA_e;
    wire signed [7:0] RD1e, RD2e; // RD1e -> SrcA_e, RD2e -> SrcB_reg_e
    wire signed [7:0] SrcB ;
    wire Zero;
    wire [7:0] ALUResult_e, ALUResult_m, ALUResult_w;
    wire [3:0] ALUControl_d, ALUControl_e;
    wire ALUSrc_d, ALUSrc_e, ResultSrc_e, ResultSrc_d, ResultSrc_m, ResultSrc_w;
    wire [7:0] pcplus1_d, pcplus1_e, pcplus1_m, pcplus1_w;
    wire [2:0] a1_d, Rs1D, Rs2D;
    wire [2:0] Rs1E, Rs2E;
    wire [2:0] a1_e, a1_m, a1_w; //Destination Register
    wire [3:0] opcode;
    wire [2:0] funct;
    wire [7:0] SrcB_reg, SrcB_reg_e, SrcB_reg_m;
    wire [7:0] wrtdata;
    wire [7:0] read_data_w, read_data_m;
    wire MemWrite_d, MemWrite_e, MemWrite_m;
    wire regwrite_d, regwrite_e, regwrite_w, regwrite_m;
    wire [7:0] ImmExt_d, ImmExt_e; //To extender
    wire [1:0] ForwardAE, ForwardBE;
    wire stallF, flushD, stallD, flushE;

    pc p0(.pcnext(pcnext_f),. reset(reset), .pc(pc_f), .stallF(stallF), .clk(CLK));

    assign pcplus1_f = pc_f + 1;
    Instruction_memory i0(.address(pc_f), .instruction(instr_f));
    assign opcode = instr_f[15:12];
    assign pcnext_f = (opcode == 4'b0101 && Zero) ? (pc_f + ImmExt_d) : 
                      (pcsrc) ? ImmExt_d :
                      (pc_f + 1);

    assign a1_d = instr_f[8:6];  //Destination register 
    assign funct = instr_f[11:9];
    assign Rs1D = instr_f[5:3];
    assign Rs2D = instr_f[2:0];
    
    Extender e0(.in0(Rs2D), .ImmExt(ImmExt_d));
    
    IF_ID l0(.clk(CLK), .flushD(flushD), .stallD(stallD), .pcplus1_f(pcnext_f), .pcplus1_d(pcplus1_d),  .reset(reset), .pc_f(pc_f), .pc_d(pc_d),  .instr_d(instr_d), .instr_f(instr_f));

    ControlUnit c0(.opcode(opcode), .funct(funct), .Zero(Zero), .ALUSrc(ALUSrc_d), .MemWrite(MemWrite_d), .ALUControl(ALUControl_d), .ResultSrc(ResultSrc_d), .PCSrc(pcsrc), .RegWrite(regwrite_d));
    Register_file r0(.clk(CLK), .reset(reset), .write(regwrite_w), .destreg(a1_w), .srcreg1(Rs1D), .srcreg2(Rs2D), .rdata1(SrcA_d), .rdata2(SrcB_reg), .wrtData(wrtdata));

    ID_EX l1(.clk(CLK), .flushE(flushE), .reset(reset), .SrcA(SrcA_d), .SrcB_reg(SrcB_reg), 
        .pcplus1(pcplus1_d), .ImmExt(ImmExt_d), 
        .destreg(a1_d), .Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E),
        .RegWrite(regwrite_d), .ALUSrc(ALUSrc_d), .ResultSrc(ResultSrc_d), 
        .MemWrite(MemWrite_d), .ALUControl(ALUControl_d), 
        .SrcA_out(RD1e),
        .SrcB_reg_out(RD2e), 
        .pcplus1_out(pcplus1_e),
        .ImmExt_out(ImmExt_e), .destreg_out(a1_e), 
        .RegWrite_out(regwrite_e), .ALUSrc_out(ALUSrc_e), 
        .ResultSrc_out(ResultSrc_e), 
        .MemWrite_out(MemWrite_e), 
        .ALUControl_out(ALUControl_e));

    Forwarding_Mux m0(.Src_in(RD1e), .Result(wrtdata), .ALU_Result(ALUResult_m), .forward(ForwardAE), .Src_out(SrcA_e));
    Forwarding_Mux m1(.Src_in(RD2e), .Result(wrtdata), .ALU_Result(ALUResult_m), .forward(ForwardBE), .Src_out(SrcB_reg_e));

    MUX_2x1 alu(.in0(SrcB_reg_e), .in1(ImmExt_e), .select(ALUSrc_e), .out0(SrcB));
    ALU a0( .SrcA(SrcA_e), .SrcB(SrcB), .Zero(Zero), .ALUResult(ALUResult_e), .ALUControl(ALUControl_e));

    EX_MEM l2(.clk(CLK), .reset(reset), 
        .ALUResult(ALUResult_e), .SrcB_reg(SrcB_reg_e),
        .pcplus1(pcplus1_e), 
        .RegWrite(regwrite_e), .MemWrite(MemWrite_e),
        .ResultSrc(ResultSrc_e), 
        .ALUResult_out(ALUResult_m), .destreg(a1_e), 
        .WriteData_out(SrcB_reg_m ), .pcplus1_out(pcplus1_m), .destreg_out(a1_m), .RegWrite_out(regwrite_m), .MemWrite_out(MemWrite_m), .ResultSrc_out(ResultSrc_m));

    DataMemory d0(.CLK(CLK), .reset(reset), .MemWrite(MemWrite_m), .ALUResult(ALUResult_m), .WriteData(SrcB_reg_m), .ReadData(read_data_m));

    MEM_WB l3(.clk(CLK), .reset(reset), .ReadData(read_data_m), .ALUResult(ALUResult_m), .pcplus1(pcplus1_m), .destreg(a1_m), .RegWrite(regwrite_m), .ResultSrc(ResultSrc_m), .ReadData_out(read_data_w), .ALUResult_out(ALUResult_w), .pcplus1_out(pcplus1_w), .destreg_out(a1_w), .RegWrite_out(regwrite_w), .ResultSrc_out(ResultSrc_w));
    
    MUX_2x1 read(.in0(ALUResult_w), .in1(read_data_w), .select(ResultSrc_w), .out0(wrtdata));
    assign result = wrtdata;

    Hazard_unit h0(.Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), .a1_d(a1_d), .a1_e(a1_e), .a1_w(a1_w), .a1_m(a1_m), .opcode(opcode), .regwrite_m(regwrite_m), .regwrite_w(regwrite_w), .ResultSrc_e(ResultSrc_e), .pcsrc(pcsrc), .StallF(stallF), .StallD(stallD), .FlushD(flushD), .FlushE(flushE), .ForwardAE(ForwardAE), .ForwardBE(ForwardBE));

endmodule
 
module ALU_Instructions;
    reg CLK;
    reg reset;
    wire [7:0] result;
    
    Pipelined_Processor uut (
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
        CLK = 0;
        forever #5 CLK = ~CLK; 
    end
    
    integer i;
    initial begin
        #10;
        
        for (i = 0; i <=15 ; i = i+1) begin
            @(posedge CLK);
            @(posedge CLK);
            #1;
        end
        
        #200;
        
    end
endmodule