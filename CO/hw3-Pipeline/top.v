// top
`include "PC.v"
`include "ADD.v"
`include "IF_ID.v"
`include "ID_EX.v"
`include "EX_M.v"
`include "M_WB.v"
`include "FU.v"
`include "HDU.v"
`include "Controller.v"
`include "RegFile.v"
`include "SignExtend.v"
`include "Mux2to1.v"
`include "Mux4to1.v"
`include "Jump_Ctrl.v"
`include "ALU.v"


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
	parameter mem_size = 16;
	parameter pc_size = 18;

	input  clk, rst;
	
	// Instruction Memory
	output [mem_size-1:0] IM_Address;	
	input  [data_size-1:0] Instruction;

	// Data Memory
	output [mem_size-1:0] DM_Address;
	output DM_enable;
	output [data_size-1:0] DM_Write_Data;	
	input  [data_size-1:0] DM_Read_Data;
	
	// write your code here

	///////////////////////////////////////////////////////
	// Wire Declaration
	///////////////////////////////////////////////////////
	// PC
	wire [pc_size-1:0] PCout;
	wire [pc_size-1:0] PC_add4;
	
	// IF/ID pipe output
	wire [pc_size-1:0]   ID_PC;
	wire [data_size-1:0] ID_ir;

	// HDU
	wire PCWrite;
	wire IF_IDWrite;
	wire IF_Flush;
	wire ID_Flush;
	wire [4:0] ID_Rs = ID_ir[25:21];
	wire [4:0] ID_Rt = ID_ir[20:16];

	// Controller
	wire [5:0] opcode;
	wire [5:0] funct;

	wire RegDst;      // select rt(0) or rd(1) field
	wire RegWrite;
	wire Reg_imm;     // aka. ALUSrc, select reg data(0) or imm data(1)
	wire [3:0] ALUOp; // aka. ALU Ctrl
	wire MemWrite;
	wire MemtoReg;    // select ALU result(0) or mem data(1) to write to regfile
	wire Branch;
	wire Jump;
	wire Jal;
	wire Jr;
	wire lw_or_lh;
	wire sw_or_sh;

	// RegFile
	wire [4:0] Rd;
	wire [4:0] Rs;
	wire [4:0] Rt;
	wire [data_size-1:0] Rs_data;
	wire [data_size-1:0] Rt_data;
	wire [4:0] shamt;

	// Sign Extend
	wire [15:0] imm;
	wire [data_size-1:0] se_imm;
	
	// ID MUX output
	wire [4:0] Rt_Rd_out;
	wire [4:0] ID_WR_out;

	///////////////////////////////////////////////////////
	// ID/EX register and EX stage
	// WB
	wire EX_MemtoReg;
	wire EX_RegWrite;
	// M
	wire EX_MemWrite;
	wire EX_Jal;
	wire EX_lw_or_lh;
	wire EX_sw_or_sh;
	// EX
	wire EX_Reg_imm;
	wire EX_Branch;
	wire EX_Jump;
	wire EX_Jr;
	// pipe
	wire [pc_size-1:0] EX_PC;
	wire [3:0] EX_ALUOp;
	wire [4:0] EX_shamt;
	wire [data_size-1:0] EX_Rs_data;
	wire [data_size-1:0] EX_Rt_data;
	wire [data_size-1:0] EX_se_imm;
	wire [4:0] EX_WR_out;
	wire [4:0] EX_Rs;
	wire [4:0] EX_Rt;

	// input of ID/EX register
	// WB
	wire ID_MemtoReg = MemtoReg;
	wire ID_RegWrite = RegWrite;
	// M
	wire ID_MemWrite = MemWrite;
	wire ID_Jal = Jal;
	wire ID_lw_or_lh = lw_or_lh;
	wire ID_sw_or_sh = sw_or_sh;
	// EX
	wire ID_Reg_imm = Reg_imm;
	wire ID_Branch = Branch;
	wire ID_Jump = Jump;
	wire ID_Jr = Jr;
	// pipe
	// wire [pc_size-1:0] ID_PC;
	wire [3:0] ID_ALUOp = ALUOp;
	wire [4:0] ID_shamt = shamt;
	wire [data_size-1:0] ID_Rs_data = Rs_data;
	wire [data_size-1:0] ID_Rt_data = Rt_data;
	wire [data_size-1:0] ID_se_imm = se_imm;
	// wire [4:0] ID_WR_out;
	// wire [4:0] ID_Rs;
	// wire [4:0] ID_Rt;
	// end input of ID/EX register

	// Jump Part
	wire [pc_size-1:0] BranchAddr;
	wire [pc_size-1:0] JumpAddr;
	wire [pc_size-1:0] PCin;
	wire [1:0] EX_JumpOP;

	// Forwarding Unit
	wire s_ForwardRs;
	wire s_ForwardRt;
	wire ForwardA;
	wire ForwardB;

	wire [data_size-1:0] ForwardA_data;
	wire [data_size-1:0] ForwardB_data;
	wire [data_size-1:0] ForwardRs_data;
	wire [data_size-1:0] ForwardRt_data;

	// ALU part
	wire [data_size-1:0] src2;
	wire [data_size-1:0] EX_ALU_result;
	wire EX_Zero;

	///////////////////////////////////////////////////////
	// EX/M register and M stage
	// WB		  
	wire M_MemtoReg;
	wire M_RegWrite;
	// M	
	wire M_MemWrite;
	wire M_Jal;
	wire M_lw_or_lh;
	wire M_sw_or_sh;
	// pipe		  
	wire [data_size-1:0] M_ALU_result;
	wire [data_size-1:0] M_Rt_data;
	wire [pc_size-1:0] M_PCplus8;
	wire [4:0] M_WR_out;

	// input
	wire [pc_size-1:0] EX_PCplus8;

	// M Jal Part
	wire [data_size-1:0] M_WD_out;

	///////////////////////////////////////////////////////
	// M/WB register and WB stage
	// WB
	wire WB_MemtoReg;
	wire WB_RegWrite;
	// pipe
	wire [data_size-1:0] WB_DM_Read_Data;
	wire [data_size-1:0] WB_WD_out;
	wire [4:0] WB_WR_out;

	wire [data_size-1:0] M_PCplus8_32bit;
	assign M_PCplus8_32bit = 32'h00000000 | M_PCplus8;

	// input  (output of the MUX_lw_or_lh)
	wire [data_size-1:0] M_DM_Read_Data;
	// output
	// WD or DM Read data mux out
	wire [data_size-1:0] WB_Final_WD_out;


	///////////////////////////////////////////////////////
	// Wire Connection
	///////////////////////////////////////////////////////
	// PC
	assign IM_Address = PCout[pc_size-1:2];
	// Controller
	assign opcode = ID_ir[31:26];
	assign funct  = ID_ir[5:0];
	// RegFile
	assign Rs = ID_ir[25:21];
	assign Rt = ID_ir[20:16];
	assign Rd = ID_ir[15:11];
	// Sign Extend
	assign imm = ID_ir[15:0]; // for jump
	// shamt to ID/EX pipeline
	assign shamt = ID_ir[10:6];
	// Jump
	assign JumpAddr = {EX_se_imm[15:0], 2'b0}; // imm left shift 2
	// Data Memory
	assign DM_Address = M_ALU_result[17:2];
	assign DM_enable  = M_MemWrite;
	// assign DM_Write_Data;


	///////////////////////////////////////////////////////
	// module instantiation and wire connection
	///////////////////////////////////////////////////////
	
	///////////////////////////////////////////////////////
	// IF Stage
	// PC
	PC myPC (
		.clk(clk),
		.rst(rst),
		.PCWrite(PCWrite),
		.PCin(PCin),
		.PCout(PCout)
	);

	ADD #(pc_size) ADD_Plus4 (
		.A(PCout),
		.B(18'd4),
		.Cout(PC_add4)
	);

	///////////////////////////////////////////////////////
	// ID Stage
	// IF/ID pipeline register
	IF_ID myIF_ID (
		.clk(clk),
		.rst(rst),
		.IF_IDWrite(IF_IDWrite),
		.IF_Flush(IF_Flush),
		.IF_PC(PC_add4),
		.IF_ir(Instruction),
		.ID_PC(ID_PC),
		.ID_ir(ID_ir)
	);

	// Hazard Detection Unit
	HDU myHDU (
		// input
		.ID_Rs(ID_Rs),
		.ID_Rt(ID_Rt),
		.EX_WR_out(EX_WR_out),
		.EX_MemtoReg(EX_MemtoReg),
		.EX_JumpOP(EX_JumpOP),
		// output
		.PCWrite(PCWrite),
		.IF_IDWrite(IF_IDWrite),
		.IF_Flush(IF_Flush),
		.ID_Flush(ID_Flush)
	);

	// Controller
	Controller myController (
		.opcode(opcode),
		.funct(funct),
		.RegDst(RegDst),
		.RegWrite(RegWrite),
		.ALUSrc(Reg_imm),
		.ALUOp(ALUOp),
		.MemWrite(MemWrite), // MemRead signal is not used here
		.MemtoReg(MemtoReg),
		.Branch(Branch),
		.Jump(Jump),
		.Jal(Jal),
		.Jr(Jr),
		.lw_or_lh(lw_or_lh),
		.sw_or_sh(sw_or_sh)
	);

	// RegFile
	Regfile myRegfile (
		.clk(clk), 
		.rst(rst),
		.Read_addr_1(Rs),
		.Read_addr_2(Rt),
		.Read_data_1(Rs_data),
		.Read_data_2(Rt_data),
		.RegWrite(WB_RegWrite),
		.Write_addr(WB_WR_out),
		.Write_data(WB_Final_WD_out)
	);

	// Sign Extend
	SignExtend ID_se (
		.in(imm),
		.out(se_imm)
	);

	// MUX in ID stage
	Mux2to1 #(5) Mux_RegDst (
		.I0(Rt),
		.I1(Rd),
		.S(RegDst),
		.out(Rt_Rd_out)
	);

	Mux2to1 #(5) MUX_RtRd_or_Reg31 (
		.I0(Rt_Rd_out),
		.I1(5'd31),
		.S(Jal),
		.out(ID_WR_out)
	);

	///////////////////////////////////////////////////////
	// EX Stage
	// ID/EX pipeline register
	ID_EX myID_EX (
		.clk(clk),
		.rst(rst),
		// input 
		.ID_Flush(ID_Flush),
		// WB
		.ID_MemtoReg(ID_MemtoReg),
		.ID_RegWrite(ID_RegWrite),
		// M
		.ID_MemWrite(ID_MemWrite),
		.ID_Jal(ID_Jal),
		.ID_lw_or_lh(ID_lw_or_lh),
		.ID_sw_or_sh(ID_sw_or_sh),
		// EX
		.ID_Reg_imm(ID_Reg_imm),
		.ID_Branch(ID_Branch),
		.ID_Jump(ID_Jump),
		.ID_Jr(ID_Jr),
		// pipe
		.ID_PC(ID_PC),
		.ID_ALUOp(ID_ALUOp),
		.ID_shamt(ID_shamt),
		.ID_Rs_data(ID_Rs_data),
		.ID_Rt_data(ID_Rt_data),
		.ID_se_imm(ID_se_imm),
		.ID_WR_out(ID_WR_out),
		.ID_Rs(ID_Rs),
		.ID_Rt(ID_Rt),
		// output
		// WB
		.EX_MemtoReg(EX_MemtoReg),
		.EX_RegWrite(EX_RegWrite),
		// M
		.EX_MemWrite(EX_MemWrite),
		.EX_Jal(EX_Jal),
		.EX_lw_or_lh(EX_lw_or_lh),
		.EX_sw_or_sh(EX_sw_or_sh),
		// EX
		.EX_Reg_imm(EX_Reg_imm),
		.EX_Branch(EX_Branch),
		.EX_Jump(EX_Jump),
		.EX_Jr(EX_Jr),
		// pipe
		.EX_PC(EX_PC),
		.EX_ALUOp(EX_ALUOp),
		.EX_shamt(EX_shamt),
		.EX_Rs_data(EX_Rs_data),
		.EX_Rt_data(EX_Rt_data),
		.EX_se_imm(EX_se_imm),
		.EX_WR_out(EX_WR_out),
		.EX_Rs(EX_Rs),
		.EX_Rt(EX_Rt)
	);

	// Jump part
	// Branch address adder
	ADD #(pc_size) ADD_Branch (
		.A(EX_PC),
		.B({EX_se_imm[15:0], 2'b00}),
		.Cout(BranchAddr)
	);

	// Jump Control
	Jump_Ctrl myJump_Ctrl (
		.Branch(EX_Branch),
		.Zero(EX_Zero),
		.Jr(EX_Jr),
		.Jump(EX_Jump),
		.JumpOP(EX_JumpOP)
	);

	// Jump Mux
	Mux4to1 #(pc_size) MUX_JumpOP (
		.I0(PC_add4),
		.I1(BranchAddr),
		.I2(JumpAddr),
		.I3(ForwardA_data[17:0]),
		.S(EX_JumpOP),
		.out(PCin)
	);

	// Forwarding unit
	FU myFU (
		.EX_Rs(EX_Rs),
		.EX_Rt(EX_Rt),
		.M_RegWrite(M_RegWrite),
		.M_WR_out(M_WR_out),
		.WB_RegWrite(WB_RegWrite),
		.WB_WR_out(WB_WR_out),
		// outpute
		.s_ForwardRs(s_ForwardRs), // select EX/MEM.Rs or MEM/WB.Rs
		.s_ForwardRt(s_ForwardRt), // select EX/MEM.Rt or MEM/WB.Rt
		.ForwardA(ForwardA),
		.ForwardB(ForwardB)
	);

	// MUX select forward data from M or WB (the Rs part)
	Mux2to1 #(data_size) Mux_ForwardRs ( 
		.I0(M_WD_out),
		.I1(WB_Final_WD_out),
		.S(s_ForwardRs),
		.out(ForwardRs_data)
	);

	// MUX select forward data from M or WB (the Rt part)
	Mux2to1 #(data_size) Mux_ForwardRt ( 
		.I0(M_WD_out),
		.I1(WB_Final_WD_out),
		.S(s_ForwardRt),
		.out(ForwardRt_data)
	);

	// MUX select origin Rs or the forward data (the Rs part)
	Mux2to1 #(data_size) Mux_ForwardA ( 
		.I0(EX_Rs_data),
		.I1(ForwardRs_data),
		.S(ForwardA),
		.out(ForwardA_data)
	);

	// MUX select origin Rs or the forward data (the Rt part)
	Mux2to1 #(data_size) Mux_ForwardB ( 
		.I0(EX_Rt_data),
		.I1(ForwardRt_data),
		.S(ForwardB),
		.out(ForwardB_data)
	);

	// ALU part
	Mux2to1 #(data_size) MUX_reg_imm (
		.I0(ForwardB_data),
		.I1(EX_se_imm),
		.S(EX_Reg_imm),
		.out(src2)
	);

	ALU myALU (
		.ALUOp(EX_ALUOp),
		.src1(ForwardA_data),
		.src2(src2),
		.shamt(EX_shamt),
		.ALU_result(EX_ALU_result),
		.Zero(EX_Zero)
	);

	// PC Plus 8
	ADD #(pc_size) ADD_Plus8 (
		.A(EX_PC[17:0]),
		.B(18'd4),
		.Cout(EX_PCplus8)
	);

	///////////////////////////////////////////////////////
	// M Stage
	// EX/M pipeline register
	EX_M myEX_M (
		.clk(clk),
		.rst(rst),
		// input 
		// WB
		.EX_MemtoReg(EX_MemtoReg),
		.EX_RegWrite(EX_RegWrite),
		// M
		.EX_MemWrite(EX_MemWrite),
		.EX_Jal(EX_Jal),
		.EX_lw_or_lh(EX_lw_or_lh),
		.EX_sw_or_sh(EX_sw_or_sh),
		// pipe
		.EX_ALU_result(EX_ALU_result),
		.EX_Rt_data(ForwardB_data),
		.EX_PCplus8(EX_PCplus8),
		.EX_WR_out(EX_WR_out),
		// output
		// WB
		.M_MemtoReg(M_MemtoReg),
		.M_RegWrite(M_RegWrite),
		// M
		.M_MemWrite(M_MemWrite),
		.M_Jal(M_Jal),
		.M_lw_or_lh(M_lw_or_lh),
		.M_sw_or_sh(M_sw_or_sh),
		// pipe
		.M_ALU_result(M_ALU_result),
		.M_Rt_data(M_Rt_data),
		.M_PCplus8(M_PCplus8),
		.M_WR_out(M_WR_out)	
	);

	Mux2to1 #(data_size) MUX_sw_or_sh (
		.I0(M_Rt_data),
		.I1({{16{M_Rt_data[15]}}, M_Rt_data[15:0]}),
		.S(M_sw_or_sh),
		.out(DM_Write_Data)
	);

	Mux2to1 #(data_size) MUX_lw_or_lh (
		.I0(DM_Read_Data),
		.I1({{16{DM_Read_Data[15]}}, DM_Read_Data[15:0]}),
		.S(M_lw_or_lh),
		.out(M_DM_Read_Data)
	);

	Mux2to1 #(data_size) MUX_Jal_or_ALUResult (
		.I0(M_ALU_result),
		.I1(M_PCplus8_32bit),
		.S(M_Jal),
		.out(M_WD_out)
	);

	///////////////////////////////////////////////////////
	// WB Stage
	// M/WB pipeline register
	M_WB myM_WB ( 
		.clk(clk),
		.rst(rst),
		// input 
		// WB
		.M_MemtoReg(M_MemtoReg),
		.M_RegWrite(M_RegWrite),
		// pipe
		.M_DM_Read_Data(M_DM_Read_Data),
		.M_WD_out(M_WD_out),
		.M_WR_out(M_WR_out),
		// output
		// WB
		.WB_MemtoReg(WB_MemtoReg),
		.WB_RegWrite(WB_RegWrite),
		// pipe
		.WB_DM_Read_Data(WB_DM_Read_Data),
		.WB_WD_out(WB_WD_out),
    	.WB_WR_out(WB_WR_out)
	);

	Mux2to1 #(data_size) MUX_MemtoReg (
		.I0(WB_WD_out),
		.I1(WB_DM_Read_Data),
		.S(WB_MemtoReg),
		.out(WB_Final_WD_out)
	);

endmodule