`timescale 1ns / 1ps


module Pipelined_Processor (
    input CLK,
    input reset,
    output [7:0] result
);

// Declaration of Wires
wire pcsrc;
wire [7:0] pcnext_f, pc_f, pc_d;
wire [15:0] instr_f, instr_d;
wire signed [7:0] SrcA_d, SrcA_e;
wire signed [7:0] RD1e, RD2e;
wire signed [7:0] SrcB, SrcB_reg, SrcB_reg_e, SrcB_reg_m;
wire Zero;
wire [7:0] ALUResult_e, ALUResult_m, ALUResult_w;
wire [3:0] ALUControl_d, ALUControl_e;
wire ALUSrc_d, ALUSrc_e;
wire ResultSrc_d, ResultSrc_e, ResultSrc_m, ResultSrc_w;
wire [7:0] pcplus1_d, pcplus1_e, pcplus1_m, pcplus1_w;
wire [2:0] a1_d, a1_e, a1_m, a1_w;  // Destination registers
wire [2:0] Rs1D, Rs2D, Rs1E, Rs2E;
wire [3:0] opcode;
wire [2:0] funct;
wire [7:0] read_data_w, read_data_m;
wire MemWrite_d, MemWrite_e, MemWrite_m;
wire regwrite_d, regwrite_e, regwrite_m, regwrite_w;
wire [7:0] ImmExt_d, ImmExt_e;
wire [1:0] ForwardAE, ForwardBE;
wire StallF, StallD, FlushD, FlushE;
wire is_matrix_mult_f, is_matrix_mult_d, is_matrix_mult_e, is_matrix_mult_m; //NEW

wire done;
wire [31:0] C;

wire [7:0] wb_wrtdata_normal;   
wire [2:0] wb_destreg_normal;  
wire       wb_write_normal;   

wire [7:0] wb_wrtdata_matrix;   
wire [2:0] wb_destreg_matrix;  
wire       wb_write_matrix;    

wire [7:0] wrtdata_final;       
wire [2:0] destreg_final;     
wire       write_final;        
wire matrix_write_in_progress_signal;


Matrix_Output mat_reg(
    .clk(CLK),
    .reset(reset),
    .C(C),
    .is_matrix_mult(is_matrix_mult_f),
    .matrix_write_in_progress(matrix_write_in_progress_signal),
    .destreg(wb_destreg_matrix),
    .wrtdata(wb_wrtdata_matrix),
    .done(done)
);

Matrix_mult mmult (
    .A(A),
    .B(B),
    .C(C),
    .done(done)
);


MUX_2x1 read (
    .in0(ALUResult_w),
    .in1(read_data_w),
    .select(ResultSrc_w),
    .out0(wb_wrtdata_normal)
);

assign wb_destreg_normal = a1_w;
assign wb_write_normal = regwrite_w;  


assign wb_write_matrix =
                         done ? 1'b0 : matrix_write_in_progress_signal; 

MUX_2x1_wb wb_mux (
    .wrtdata0(wb_wrtdata_normal),
    .destreg0(wb_destreg_normal),
    .write0(wb_write_normal),

    .wrtdata1(wb_wrtdata_matrix),
    .destreg1(wb_destreg_matrix),
    .write1(matrix_write_in_progress_signal),

    .sel(matrix_write_in_progress_signal), 

    .wrtdata_out(wrtdata_final),
    .destreg_out(destreg_final),
    .write_out(write_final)
);


Register_file r0 (
    .clk(CLK),
    .reset(reset),
    .write(write_final),
    .destreg(destreg_final),
    .srcreg1(Rs1D),
    .srcreg2(Rs2D),
    .rdata1(SrcA_d),
    .rdata2(SrcB_reg),
    .wrtData(wrtdata_final),
    .ReadDataA0(ReadDataA0),
    .ReadDataA1(ReadDataA1),
    .ReadDataA2(ReadDataA2),
    .ReadDataA3(ReadDataA3),
    .ReadDataB0(ReadDataB0),
    .ReadDataB1(ReadDataB1),
    .ReadDataB2(ReadDataB2),
    .ReadDataB3(ReadDataB3)
);

// IF Stage
pc p0 (
    .pcnext(pcnext_f),
    .reset(reset),
    .pc(pc_f),
    .StallF(StallF),
    .clk(CLK)
);

Instruction_memory i0 (
    .address(pc_f),
    .instruction(instr_f)
);

assign opcode = instr_f[15:12];

// PC MUX
assign pcnext_f = (opcode == 4'b0101 && Zero) ? (pc_f + ImmExt_d) :
                  (pcsrc) ? ImmExt_d :
                  (pc_f + 1);

// ID Stage
assign a1_d = instr_f[8:6];
assign funct = instr_f[11:9];
assign Rs1D = instr_f[5:3];
assign Rs2D = instr_f[2:0];

// Immediate Extender
Extender e0(
    .in0(Rs2D),
    .ImmExt(ImmExt_d)
);

// IF/ID Pipeline Register
IF_ID l0 (
    .clk(CLK),
    .FlushD(FlushD),
    .StallD(StallD),
    .reset(reset),
    .pcplus1_f(pcnext_f),
    .pcplus1_d(pcplus1_d),
    .pc_f(pc_f),
    .pc_d(pc_d),
    .instr_f(instr_f),
    .instr_d(instr_d),
    .is_matrix_mult_f(is_matrix_mult_f),
    .is_matrix_mult_d(is_matrix_mult_d)
);

// Control Unit
ControlUnit c0 (
    .opcode(opcode),
    .funct(funct),
    .Zero(Zero),
    .ALUSrc(ALUSrc_d),
    .MemWrite(MemWrite_d),
    .ALUControl(ALUControl_d),
    .ResultSrc(ResultSrc_d),
    .PCSrc(pcsrc),
    .RegWrite(regwrite_d),
    .is_matrix_mult(is_matrix_mult_f)
);

// Register File reading handled above

// EX Stage
ID_EX l1 (
    .clk(CLK),
    .FlushE(FlushE),
    .reset(reset),
    .SrcA(SrcA_d),
    .SrcB_reg(SrcB_reg),
    .pcplus1(pcplus1_d),
    .ImmExt(ImmExt_d),
    .destreg(a1_d),
    .Rs1D(Rs1D),
    .Rs2D(Rs2D),
    .Rs1E(Rs1E),
    .Rs2E(Rs2E),
    .RegWrite(regwrite_d),
    .ALUSrc(ALUSrc_d),
    .ResultSrc(ResultSrc_d),
    .MemWrite(MemWrite_d),
    .ALUControl(ALUControl_d),
    .SrcA_out(RD1e),
    .SrcB_reg_out(RD2e),
    .pcplus1_out(pcplus1_e),
    .ImmExt_out(ImmExt_e),
    .destreg_out(a1_e),
    .RegWrite_out(regwrite_e),
    .ALUSrc_out(ALUSrc_e),
    .ResultSrc_out(ResultSrc_e),
    .MemWrite_out(MemWrite_e),
    .ALUControl_out(ALUControl_e),
    .is_matrix_mult_d(is_matrix_mult_d),
    .is_matrix_mult_e(is_matrix_mult_e)
);

// Forwarding MUXes for SrcA and SrcB
Forwarding_Mux m0 (.Src_in(RD1e),
.Result(write_final),
.ALU_Result(ALUResult_m),
.forward(ForwardAE),
.Src_out(SrcA_e));

Forwarding_Mux m1 (.Src_in(RD2e),
.Result(write_final),
.ALU_Result(ALUResult_m),
.forward(ForwardBE),
.Src_out(SrcB_reg_e));

// ALU Source selection: Immediate or Register
MUX_2x1 alu (.in0(SrcB_reg_e),
.in1(ImmExt_e),
.select(ALUSrc_e),
.out0(SrcB));

// ALU Operation
ALU a0 (
.SrcA(SrcA_e),
.SrcB(SrcB),
.Zero(Zero),
.ALUResult(ALUResult_e),
.ALUControl(ALUControl_e)

);

// MEM Stage (Memory)

// EX/MEM Pipeline Register
EX_MEM l2 (
.clk(CLK),
.reset(reset),
.ALUResult(ALUResult_e),
.SrcB_reg(SrcB_reg_e),
.pcplus1(pcplus1_e),
.RegWrite(regwrite_e),
.MemWrite(MemWrite_e),
.ResultSrc(ResultSrc_e),
.ALUResult_out(ALUResult_m),
.destreg(a1_e),
.WriteData_out(SrcB_reg_m),
.pcplus1_out(pcplus1_m),
.destreg_out(a1_m),
.RegWrite_out(regwrite_m),
.MemWrite_out(MemWrite_m),
.ResultSrc_out(ResultSrc_m),
.is_matrix_mult_e (is_matrix_mult_e),
.is_matrix_mult_m (is_matrix_mult_m)
);

// Data Memory
DataMemory d0 (
.CLK(CLK),
.reset(reset),
.MemWrite(MemWrite_m),
.ALUResult(ALUResult_m),
.WriteData(SrcB_reg_m),
.ReadData(read_data_m)
);

// WB Stage (Write-Back)


MEM_WB l3 (
.clk(CLK),
.reset(reset),
.ReadData(read_data_m),
.ALUResult(ALUResult_m),
.pcplus1(pcplus1_m),
.destreg(a1_m),
.RegWrite(regwrite_m),
.ResultSrc(ResultSrc_m),
.ReadData_out(read_data_w),
.ALUResult_out(ALUResult_w),
.pcplus1_out(pcplus1_w),
.destreg_out(a1_w),
.RegWrite_out(regwrite_w),
.ResultSrc_out(ResultSrc_w)
);


// Hazard Detection + Forwarding

Hazard_unit h0 (
.Rs1D(Rs1D),
.Rs2D(Rs2D),
.Rs1E(Rs1E),
.Rs2E(Rs2E),
.a1_d(a1_d),
.a1_e(a1_e),
.a1_w(a1_w),
.a1_m(a1_m),
.opcode(opcode),
.regwrite_m(regwrite_m),
.regwrite_w(regwrite_w),
.ResultSrc_e(ResultSrc_e),
.pcsrc(pcsrc),
.StallF(StallF),
.StallD(StallD),
.FlushD(FlushD),
.FlushE(FlushE),
.ForwardAE(ForwardAE),
.ForwardBE(ForwardBE),
.matrix_write_in_progress(matrix_write_in_progress_signal)
);


// Final result output
assign result = wrtdata_final;

endmodule
