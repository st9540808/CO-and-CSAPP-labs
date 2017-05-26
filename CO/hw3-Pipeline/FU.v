// Forwarding Unit

module FU ( // input 
            EX_Rs,
            EX_Rt,
            M_RegWrite,
            M_WR_out,
            WB_RegWrite,
            WB_WR_out,
            // output
            // write your code in here
            s_ForwardRs, // select EX/MEM.Rs or MEM/WB.Rs
            s_ForwardRt, // select EX/MEM.Rt or MEM/WB.Rt
            ForwardA,
            ForwardB
			);

	input [4:0] EX_Rs;
    input [4:0] EX_Rt;
    input M_RegWrite;
    input [4:0] M_WR_out;
    input WB_RegWrite;
    input [4:0] WB_WR_out;

	// write your code in here
	output reg s_ForwardRs;
	output reg s_ForwardRt;
	output reg ForwardA;
	output reg ForwardB;

	always @(*) begin
		ForwardA = 0;
		ForwardB = 0;
		s_ForwardRs = 0;
		s_ForwardRt = 0;

		// Rs Section
		// Forwarding from EX/MEM register
		if (M_RegWrite
			&& M_WR_out != 0
			&& EX_Rs == M_WR_out) begin
				s_ForwardRs = 0; // From MEM				
				ForwardA = 1;
		end
		// Forwarding from MEM/WB register
		else if (WB_RegWrite
			&& WB_WR_out != 0
			&& EX_Rs == WB_WR_out) begin
				s_ForwardRs = 1; // From WB			
				ForwardA = 1;
		end

		// Rt Section
		// Forwarding from EX/MEM register
		if (M_RegWrite
			&& M_WR_out != 0
			&& EX_Rt == M_WR_out) begin
				s_ForwardRt = 0; // From MEM
				ForwardB = 1;
		end
		// Forwarding from MEM/WB register
		else if (WB_RegWrite
			&& WB_WR_out != 0
			&& EX_Rt == WB_WR_out) begin
				s_ForwardRt = 1; // From WB			
				ForwardB = 1;
		end
	end

endmodule