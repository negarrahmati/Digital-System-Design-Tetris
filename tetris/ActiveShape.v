module ActiveShape(input clk, reset);
reg [3:0]R[3:0];
integer X, Y, T;
integer A, B;
integer i, j;
always @(posedge clk)
begin
	if(reset)
	begin
		X = 0; Y = 0;
		T = {$random} % 5;
		T = 5;
		case T:
			0: begin end
			1: begin end
			2: begin end
			3: begin end
			4: begin end
			5: begin for(i=0; i<4; i=i+1) for(j=0; j<4; j=j+1) R[i][j] = 1; end
			
		endcase
	end
	else begin
		X = X + 1;
		Y = Y + 1;
	end
end

endmodule
