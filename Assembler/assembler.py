opcode_map = {
    "RTYPE":        0b0000,
    "LOAD_IM":      0b0001,
    "LOAD":         0b0010,
    "STORE":        0b0011,
    "JMP":         0b0100,
    "EQUAL_TO":     0b0101,
    "RIGHT_SHIFT":  0b0110,
    "LEFT_SHIFT":   0b0111,
    "ADDI":         0b1000,
    "BEQ":          0b1001,
}
FUNCT_CODES = {
    "ADD":       0b000,
    "SUB":       0b001,
    "MUL":       0b010,
    "DIV":       0b011,
    "AND":       0b100,
    "OR":        0b101,
    "XOR":       0b110,
    "NOT":       0b111,
}

register_map = {
    f"R{i}": i for i in range(8)
}

def to_binary(value, bits):
    return format(value & ((1 << bits) - 1), f"0{bits}b")

def assemble(instruction):
    parts = instruction.strip().split()
    if not parts:
        return None

    instr = parts[0].upper()

    if instr == "ADD":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["ADD"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "SUB":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["SUB"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "MUL":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["MUL"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "DIV":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["DIV"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "AND":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["AND"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "OR":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["OR"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "XOR":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RTYPE"]
        funct = FUNCT_CODES["XOR"]
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(rs2, 3)
    elif instr == "SLL":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["LEFT_SHIFT"]
        funct = 0
        imm = int(parts[3])
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(imm, 3)
    elif instr == "SRA":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        rs2 = register_map[parts[3].upper()]
        op = opcode_map["RIGHT_SHIFT"]
        funct = 0
        imm = int(parts[3])
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(imm, 3)
    elif instr == "ADDI":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        imm = int(parts[3])
        op = opcode_map[instr]
        funct = 0
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(imm, 3)
    elif instr == "STORE":
        rd = register_map[parts[1].upper()]
        rs1 = register_map[parts[2].upper()]
        imm = int(parts[3])
        op = opcode_map[instr]
        funct = 0
        binary = to_binary(op, 4) + to_binary(funct, 3) + to_binary(rd, 3) + to_binary(rs1, 3) + to_binary(imm, 3)
    elif instr == "JMP":
        addr = int(parts[1])
        op = opcode_map[instr]
        binary = to_binary(op, 4)  + to_binary(addr, 12)
    else:
        print("Unsupported instruction.")
        return None

    hex_code = hex(int(binary, 2))[2:].zfill(4).upper()
    print("Binary:", binary)
    print("Hex   :", hex_code)
    return hex_code


if __name__ == "__main__":
    while True:
        try:
            line = input("Enter instruction (or 'exit'): ")
            if line.strip().lower() == "exit":
                break
            assemble(line)
        except Exception as e:
            print("Error:", e)