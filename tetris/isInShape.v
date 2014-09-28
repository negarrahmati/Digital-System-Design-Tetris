module isInShape(output reg inshape, input X, input Y, input Coord_X, input Coord_Y, input Type);
	initial inshape=0;
	always @(Coord_X or Coord_Y) begin
		if ( Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20 )
			inshape = 1'b1;
		else
			inshape = 1'b0;
	end

endmodule