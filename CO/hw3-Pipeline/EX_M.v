// EX_M

module EX_M ( clk,
			  rst,
			  // input 
			  // WB
			  EX_MemtoReg,
			  EX_RegWrite,
			  // M
			  EX_MemWrite,
			  EX_Jal,
			  EX_lw_or_lh,
			  EX_sw_or_sh,
			  // pipe
			  EX_ALU_result,
			  EX_Rt_data,
			  EX_PCplus8,
			  EX_WR_out,
			  // output
			  // WB
			  M_MemtoReg,
			  M_RegWrite,
			  // M
			  M_MemWrite,
			  M_Jal,
			  M_lw_or_lh,
			  M_sw_or_sh,
			  // pipe
			  M_ALU_result,
			  M_Rt_data,
			  M_PCplus8,
			  M_WR_out			  		  			  
			  );
	
	parameter pc_size = 18;	
	parameter data_size = 32;
	
	input clk, rst;
			  
	// WB		  
	input EX_MemtoReg;
	input EX_RegWrite;
	// M
	input EX_MemWrite;
	input EX_Jal;
	input EX_lw_or_lh;
	input EX_sw_or_sh;
	// pipe		  
	input [data_size-1:0] EX_ALU_result;
	input [data_size-1:0] EX_Rt_data;
	input [pc_size-1:0] EX_PCplus8;
	input [4:0] EX_WR_out;
	
	// WB
	output M_MemtoReg;	
	output M_RegWrite;	
	// M	
	output M_MemWrite;	
	output M_Jal;
	output M_lw_or_lh;
	output M_sw_or_sh;
	// pipe		  
	output [data_size-1:0] M_ALU_result;
	output [data_size-1:0] M_Rt_data;
	output [pc_size-1:0] M_PCplus8;
	output [4:0] M_WR_out;
	
	// write your code in here
	// WB
	reg M_MemtoReg;	
	reg M_RegWrite;	
	// M	
	reg M_MemWrite;
	reg M_Jal;
	reg M_lw_or_lh;
	reg M_sw_or_sh;
	// write your code in here
	// pipe		  
	reg [data_size-1:0] M_ALU_result;
	reg [data_size-1:0] M_Rt_data;
	reg [pc_size-1:0] M_PCplus8;
	reg [4:0] M_WR_out;

	always @(negedge clk or posedge rst) begin
		if (rst) begin
			// WB
			M_MemtoReg   <= 0;	
			M_RegWrite   <= 0;	
			// M
			M_MemWrite   <= 0;
			M_Jal        <= 0;
			M_lw_or_lh   <= 0;
			M_sw_or_sh   <= 0;
			// pipe
			M_ALU_result <= 0;
			M_Rt_data    <= 0;
			M_PCplus8    <= 0;
			M_WR_out     <= 0;
		end
		else begin
			// WB
			M_MemtoReg   <= EX_MemtoReg;	
			M_RegWrite   <= EX_RegWrite;	
			// M
			M_MemWrite   <= EX_MemWrite;
			M_Jal        <= EX_Jal;
			M_lw_or_lh   <= EX_lw_or_lh;
			M_sw_or_sh   <= EX_sw_or_sh;
			// pipe
			M_ALU_result <= EX_ALU_result;
			M_Rt_data    <= EX_Rt_data;
			M_PCplus8    <= EX_PCplus8;
			M_WR_out     <= EX_WR_out;
		end
	end
	
endmodule