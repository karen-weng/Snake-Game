module ifhit (X1, X2, Y1, Y2, XDIM, YDIM, hit, enable, CLOCK_50)
    input [7:0] X1, X2;
	input [6:0] Y1, Y2;
    input XDIM, YDIM;
    output hit;

    assign hit = (X1 < X2 + XDIM) && (X1 + XDIM > X2) &&
                 (Y1 < Y2 + YDIM) && (Y1 + YDIM > Y2);

endmodule 

