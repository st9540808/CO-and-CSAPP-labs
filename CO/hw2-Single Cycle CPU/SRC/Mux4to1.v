module Mux4to1 (
		I0, I1, I2, I3,
		S,
		out
	);

	parameter bit_size = 16;

	input [bit_size-1:0] I0;
	input [bit_size-1:0] I1;
	input [bit_size-1:0] I2;
	input [bit_size-1:0] I3;
	input [1:0] S;

	output reg [bit_size-1:0] out;
	
	always @(*) begin
		case (S)
			2'b00 : out = I0;
			2'b01 : out = I1;
			2'b10 : out = I2;
			2'b11 : out = I3;
		endcase
	end

endmodule