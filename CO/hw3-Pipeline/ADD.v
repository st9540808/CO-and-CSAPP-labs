module ADD (
	A,
	B,
	Cout
	);
				 
	parameter bit_size = 18;

	input  [bit_size-1:0] A;
	input  [bit_size-1:0] B;
	output [bit_size-1:0] Cout;

	assign Cout = A + B;

endmodule