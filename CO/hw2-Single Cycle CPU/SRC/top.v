// top
`include "Controller.v"
`include "Jump_Ctrl.v"
`include "ALU.v"
`include "Regfile.v"
`include "PC.v"
`include "SignExtend.v"
`include "Mux2to1.v"
`include "Mux4to1.v"

module top ( clk,
             rst,
             // Instruction Memory
             IM_Address,
             Instruction,
             // Data Memory
             DM_Address,
             DM_enable,
             DM_Write_Data,
             DM_Read_Data);

	parameter data_size = 32;
	parameter mem_size  = 16;	

	input  clk, rst;
	
	// Instruction Memory
	output [mem_size-1:0]  IM_Address;	
	input  [data_size-1:0] Instruction;

	// Data Memory
	output [mem_size-1:0]  DM_Address;
	output DM_enable; // aka. MemWrite in P&H
	output [data_size-1:0] DM_Write_Data;	
	input  [data_size-1:0] DM_Read_Data;
	
	// write your code here
	// PC
	parameter pc_size = 18;
	wire [pc_size-1:0] PCout;
	wire [pc_size-1:0] PC_add4;
	wire [pc_size-1:0] PC_add8;

	// Controller
	wire [5:0] opcode;
	wire [5:0] funct;

	wire RegDst;      // select rt(0) or rd(1) field
	wire RegWrite;
	wire ALUSrc;      // select reg data(0) or imm data(1)
	wire [3:0] ALUOp; // aka. ALU Ctrl
	wire MemtoReg;    // select ALU result(0) or mem data(1) to write to Regfile
	wire Branch;
	wire Jump;
	wire Jal;         // control two MUX2to1
	wire Jr;
	wire lw_or_lh;
	wire sw_or_sh;

	// Registers
	wire [4:0] Rs;
	wire [4:0] Rt;
	wire [4:0] Rd;
	wire [4:0] Rt_or_Rd; // ouput for MUX_RegDst

	wire [4:0] WR_in; // input for WR
	wire [data_size-1:0] Reg_WD_in; // input for WD
	wire [data_size-1:0] Rs_data;
	wire [data_size-1:0] Rt_data;

	// Sign extend and imm
	wire [15:0]          imm;
	wire [data_size-1:0] imm_sign_extended; // from imm

	// ALU
	wire [4:0] shamt;
	wire [data_size-1:0] src2;
	wire [data_size-1:0] ALU_result;
	wire Zero;

	// Jump Part
	wire [1:0] JumpOp; // aka. Jump Ctrl
	wire [pc_size-1:0] BranchAddr;
	wire [pc_size-1:0] JumpAddr;
	wire [pc_size-1:0] PCin;
	
	// Data memory
	wire [data_size-1:0] DM_RD_out;
	wire [data_size-1:0] DMdata_or_ALUResult;

	///////////////////////////////////////////////////////
	// Wire connection
	///////////////////////////////////////////////////////
	// PC
	assign IM_Address = PCout[pc_size-1:2];
	// Controller
	assign opcode = Instruction[31:26];
	assign funct  = Instruction[5:0];
	// Register
	assign Rs = Instruction[25:21];
	assign Rt = Instruction[20:16];
	assign Rd = Instruction[15:11];
	// Sign extend
	assign imm = Instruction[15:0]; // for jump
	// ALU
	assign shamt = Instruction[10:6];
	// Jump
	assign JumpAddr = {imm, 2'b0}; // imm left shift 2
	// Data memory
	assign DM_Address = ALU_result[17:2];

	///////////////////////////////////////////////////////
	// Instantiate and connect wires
	///////////////////////////////////////////////////////
	// PC
	PC myPC (
		.clk(clk),
		.rst(rst),
		.PCin(PCin),
		.PCout(PCout)
	);

	ADD #(.bit_size(pc_size)) ADD_Plus4_1 (
		.A(18'd4),
		.B(PCout),
		.Cout(PC_add4)
	);

	ADD #(.bit_size(pc_size)) ADD_Plus4_2 (
		.A(18'd4),
		.B(PC_add4),
		.Cout(PC_add8)
	);

	// Controller
	Controller myController (
		.opcode(opcode),
		.funct(funct),
		.RegDst(RegDst),
		.RegWrite(RegWrite),
		.ALUSrc(ALUSrc),
		.ALUOp(ALUOp),
		.MemWrite(DM_enable), // MemWrite to DM_enable
		.MemtoReg(MemtoReg),
		.Branch(Branch),
		.Jump(Jump),
		.Jal(Jal),
		.Jr(Jr),
		.lw_or_lh(lw_or_lh),
		.sw_or_sh(sw_or_sh)
	);

	// Registers
	Mux2to1 #(.bit_size(5)) MUX_RegDst (
		.I0(Rt),
		.I1(Rd),
		.S(RegDst),
		.out(Rt_or_Rd)
	);

	Mux2to1 #(.bit_size(5)) MUX_RtRd_or_Reg31 (
		.I0(Rt_or_Rd),
		.I1(5'd31),
		.S(Jal),
		.out(WR_in)
	);

	Regfile myRegfile (
		.clk(clk),
		.rst(rst),
		.Read_addr_1(Rs),
		.Read_addr_2(Rt),
		.Read_data_1(Rs_data),
		.Read_data_2(Rt_data),
		.RegWrite(RegWrite),
		.Write_addr(WR_in),
		.Write_data(Reg_WD_in)
	);

	// Sign extend
	SignExtend imm_SignExtender (
		.in(imm),
		.out(imm_sign_extended)
	);

	// ALU
	Mux2to1 #(.bit_size(data_size)) MUX_ALUSrc (
		.I0(Rt_data),
		.I1(imm_sign_extended),
		.S(ALUSrc),
		.out(src2)
	);

	ALU #(.bit_size(data_size)) myALU (
		.ALUOp(ALUOp),
		.src1(Rs_data),
		.src2(src2),
		.shamt(shamt),
		.ALU_result(ALU_result),
		.Zero(Zero)
	);

	// Jump part
	Jump_Ctrl myJump_Ctrl (
		.Zero(Zero),
		.JumpOP(JumpOp),
		.Branch(Branch),
		.Jump(Jump),
		.Jr(Jr)
	);

	ADD #(.bit_size(pc_size)) ADD_Branch (
		.A({imm, 2'b0}),
		.B(PC_add4),
		.Cout(BranchAddr)
	);

	Mux4to1 #(.bit_size(pc_size)) PC_Mux (
		.I0(PC_add4),              // JumpOp = 0, take PC + 4
		.I1(BranchAddr),
		.I2(JumpAddr),
		.I3(Rs_data[pc_size-1:0]),
		.S(JumpOp),
		.out(PCin)
	);

	// Data memory
	Mux2to1 #(.bit_size(data_size)) MUX_sw_or_sh ( // Write data
		.I0(Rt_data),
		.I1({{16{Rt_data[15]}}, Rt_data[15:0]}),
		.S(sw_or_sh),
		.out(DM_Write_Data)
	);

	Mux2to1 #(.bit_size(data_size)) MUX_lw_or_lh (
		.I0(DM_Read_Data),
		.I1({{16{DM_Read_Data[15]}}, DM_Read_Data[15:0]}),
		.S(lw_or_lh),
		.out(DM_RD_out)
	);

	Mux2to1 #(.bit_size(data_size)) MUX_MemtoReg (
		.I0(ALU_result),
		.I1(DM_RD_out),
		.S(MemtoReg),
		.out(DMdata_or_ALUResult)
	);

	Mux2to1 #(.bit_size(data_size)) MUX_Jal_or_MemtoReg (
		.I0(DMdata_or_ALUResult),
		.I1({14'b0, PC_add8}),
		.S(Jal),
		.out(Reg_WD_in)
	);

endmodule


// for ADD 4
module ADD (
	A,
	B,
	Cout
	);
				 
	parameter bit_size = 16;

	input  [bit_size-1:0] A;
	input  [bit_size-1:0] B;
	output [bit_size-1:0] Cout;

	assign Cout = A + B;

endmodule
