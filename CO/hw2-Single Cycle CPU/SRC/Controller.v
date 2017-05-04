// Controller

module Controller ( opcode,
                    funct,
                    // write your code in here
                    RegDst,
                    RegWrite,
                    ALUSrc,
                    ALUOp,
                    MemWrite, // MemRead signal is not used here
                    MemtoReg,
                    Branch,
                    Jump,
                    Jal,
                    Jr,
                    lw_or_lh,
                    sw_or_sh
					);

	input  [5:0] opcode;
	input  [5:0] funct;

	// write your code in here
	output reg RegDst;      // select rt(0) or rd(1) field
	output reg RegWrite;
	output reg ALUSrc;      // select reg data(0) or imm data(1)
	output reg [3:0] ALUOp; // aka. ALU Ctrl
	output reg MemWrite;
	output reg MemtoReg;    // select ALU result(0) or mem data(1) to write to regfile
	output reg Branch;
	output reg Jump;
	output reg Jal;
	output reg Jr;
	output reg lw_or_lh;
	output reg sw_or_sh;

	// ALU operation
	parameter op_nop = 0,
	          op_add = 1,
	          op_sub = 2,
	          op_and = 3,
	          op_or  = 4,
	          op_xor = 5,
	          op_nor = 6,
	          op_slt = 7,
	          op_sll = 8,
	          op_srl = 9,
	          op_beq = 10,
	          op_bne = 11;

	always @(*) begin
		RegDst   = 0; // default to write reg rt
		RegWrite = 0; // default to not doing anything
		ALUSrc   = 0; // default to read second reg data for ALU src2
		ALUOp    = 0; // default to nop
		MemWrite = 0; // default to not doing anything
		MemtoReg = 0; // default to fed the reg from ALU output
		Branch   = 0;
		Jump     = 0;
		Jal      = 0;
		Jr       = 0;
		lw_or_lh = 0; // default to lw;
		sw_or_sh = 0; // default to sw;

		case (opcode)
			// R type
			6'b00_0000 : begin
				case (funct)
					6'b10_0000 : begin // add
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_add;
					end
					6'b10_0010 : begin // sub
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_sub;
					end
					6'b10_0100 : begin // and
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_and;
					end
					6'b10_0101 : begin // or
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_or;
					end
					6'b10_0110 : begin // xor
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_xor;
					end
					6'b10_0111 : begin // nor
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_nor;
					end
					6'b10_1010 : begin // slt
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_slt;
					end
					6'b00_0000 : begin // sll
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_sll;
					end
					6'b00_0010 : begin // srl
						RegDst   = 1;
						RegWrite = 1;
						ALUOp    = op_srl;
					end
					6'b00_1000 : begin // jr
						// PC = Rs
						Jr = 1;
					end
					6'b00_1001 : begin // jalr
						// R[31] = PC + 8
						// PC = Rs
						Jr       = 1;
						Jal      = 1;						
						RegWrite = 1;
					end
					default : begin
						RegDst = 1;
						ALUOp  = op_nop;
					end
				endcase
			end
			// I type
			6'b00_1000 : begin // addi
				RegWrite = 1;
				ALUSrc   = 1;
				ALUOp    = op_add;
			end
			6'b00_1100 : begin // andi
				RegWrite = 1;
				ALUSrc   = 1;
				ALUOp    = op_and;
			end
			6'b00_1010 : begin // slti
				RegWrite = 1;
				ALUSrc   = 1;
				ALUOp    = op_slt;
			end
			6'b00_0100 : begin // beq
				Branch = 1;
				ALUOp  = op_beq;
			end
			6'b00_0101 : begin // bne
				Branch = 1;
				ALUOp  = op_bne;
			end
			6'b10_0011 : begin // lw
				ALUSrc   = 1;
				ALUOp    = op_add;
				MemtoReg = 1;
				RegWrite = 1; // MemRead  = 1; (by default)
			end
			6'b10_0001 : begin // lh
				ALUSrc   = 1;
				ALUOp    = op_add;
				MemtoReg = 1;
				RegWrite = 1;
				lw_or_lh = 1;
			end
			6'b10_1011 : begin // sw
				ALUSrc   = 1;
				ALUOp    = op_add; // MemRead  = 1; (by default)
				MemWrite = 1;
			end
			6'b10_1001 : begin // sh
				ALUSrc   = 1;
				ALUOp    = op_add;
				MemWrite = 1;
				sw_or_sh = 1;
			end
			// J type
			6'b00_0010 : begin // j
				Jump = 1;
			end
			6'b00_0011 : begin // jal
				Jump     = 1;
				Jal      = 1;
				RegWrite = 1;
			end
			default : begin
				RegDst = 0;
			end
		endcase
	end

endmodule
