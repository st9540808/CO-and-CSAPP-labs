// Hazard Detection Unit

module HDU ( // input
             ID_Rs,
             ID_Rt,
             EX_WR_out,
             EX_MemtoReg,
             EX_JumpOP,
             // output
             // write your code in here
             PCWrite,
             IF_IDWrite,
             IF_Flush,
             ID_Flush
			 );

	parameter bit_size = 32;
	
	input [4:0] ID_Rs;
	input [4:0] ID_Rt;
	input [4:0] EX_WR_out;
	input EX_MemtoReg;
	input [1:0] EX_JumpOP;
	
	// write your code in here
	output reg PCWrite;
	output reg IF_IDWrite;
	output reg IF_Flush;
	output reg ID_Flush;

	always @(*) begin
		// default signal
		PCWrite    = 1;
		IF_IDWrite = 1;
		IF_Flush   = 0;
		ID_Flush   = 0;

		// Branch
		if (EX_JumpOP == 1) begin
			IF_IDWrite = 1;
			IF_Flush   = 1;
			ID_Flush   = 1;
		end
		
		// lw hazard
		if (EX_MemtoReg
			&& (ID_Rt == EX_WR_out || ID_Rs == EX_WR_out)) begin
			PCWrite    = 0;
			IF_IDWrite = 0; // halt IF/ID regsiter
			ID_Flush   = 1;
		end
	end
	
endmodule