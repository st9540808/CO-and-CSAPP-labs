// Jump_Ctrl

module Jump_Ctrl( Zero,
                  JumpOP,
				  // write your code in here
				  Branch,
				  Jump,
				  Jr
				  );

    input Zero;
	output [1:0] JumpOP;
	
	// write your code in here
//	reg [1:0] JumpOP;
	input Branch;
	input Jump;
	input Jr;

	assign JumpOP = (Branch && Zero) ? 1 :
	                (Jump ? 2 :
	                (Jr ? 3 : 0));
	
endmodule