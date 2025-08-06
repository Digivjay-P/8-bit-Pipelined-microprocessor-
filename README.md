# 🧠 8-Bit Microprocessor Project (Pipelined + Non-Pipelined)

Welcome to the repository for my custom-designed **8-bit microprocessor system**, featuring both **single-cycle** and **pipelined** architectures. This project showcases my end-to-end implementation of a RISC-style processor, assembler, and test programs, all designed in Verilog and simulated using Vivado.

---

## 🗂️ Repository Structure

| Folder / File                                                                 | Description                                                                 |
|------------------------------------------------------------------------------|-----------------------------------------------------------------------------|
| [`Assembler`](./Assembler)                                                  | Custom assembler scripts to convert `.txt` programs into machine code       |
| [`Bonus Program 1`](./Bonus%20Program%201)                                   | Extra custom program files for testing and showcasing capabilities         |
| [`Final Synthesizable`](./Final%20Synthesizable)                             | Cleaned, final versions of synthesizable Verilog modules                    |
| [`Matrix Multiplication`](./Matrix%20Multiplication)                         | Assembly-level matrix multiplication implementation                        |
| [`Modules`](./Modules)                                                      | Modular Verilog components (ALU, Control Unit, Register File, etc.)        |
| [`NonPipelined/Single_cyclemicroprocessor`](./NonPipelined)            | Fully working 8-bit non-pipelined processor design                          |
| [`Pipelined Processor with Hazard Unit`](./Pipelined%20Processor%20with%20Hazard%20Unit) | Pipelined processor with forwarding, stalling & hazard detection |
| [`README.md`](./README.md)                                                  | You're here!                                                                |

---

## 🚀 Features

### ✅ Core Features

- **8-bit Custom ISA**
- **Assembler**: Written in Python, converts assembly code to binary machine code
- **Single-Cycle Processor**: Fully functional, with ALU, memory, and control logic
- **Pipelined Processor**: Includes 5-stage pipeline with:
  - Instruction Fetch (IF)
  - Instruction Decode (ID)
  - Execute (EX)
  - Memory Access (MEM)
  - Write-back (WB)
- **Hazard Detection Unit**: Implements data forwarding and stalling

### 📈 Additional Programs

- Matrix Multiplication (Assembly)
- Bonus programs for extended testing

---

## 🛠️ Tools Used

- **Verilog HDL** – Hardware description and simulation
- **Python** – For assembler scripting
- **Vivado** – Synthesis and simulation
- **Git** – Version control

---

## 🔧 Usage Instructions

1. **Assemble your program:**

```bash
cd Assembler/
python3 assembler.py program.asm
