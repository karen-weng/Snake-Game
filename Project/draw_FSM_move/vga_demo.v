/*
*   This code draws a horizontal line on the screen and then moves the line up and down. The
*   line "bounces" off the top and bottom of the screen and reverses directions. To run the demo
*   first press/release KEY[0] to reset the circuit. Then, press/release KEY[1] to initialize
*   the (x,y) location of the line. The line color is determined by SW[2:0]. Finally, press 
*   KEY[3] to start the animation. 
*/
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
	
    parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011; 
    parameter E = 3'b100, F = 3'b101, G = 3'b110, H = 3'b111; 
    parameter XSCREEN = 160, YSCREEN = 120;
    parameter XDIM = XSCREEN>>1, YDIM = 1;
    parameter X0 = 8'd39, Y0 = 7'd59;
    parameter ALT = 3'b000; // alternate object color
    parameter K = 2; // animation speed: use 20 for hardware, 2 for ModelSim

	input CLOCK_50;	
	input [7:0] SW;
	input [3:0] KEY;
	output [7:0] VGA_R;
	output [7:0] VGA_G;
	output [7:0] VGA_B;
	output VGA_HS;
	output VGA_VS;
	output VGA_BLANK_N;
	output VGA_SYNC_N;
	output VGA_CLK;	

	wire [7:0] VGA_X; 
	wire [6:0] VGA_Y;  
	reg [2:0] VGA_COLOR;
    reg plot;
    
	wire [2:0] colour;
	wire [7:0] X, Z;
	wire [6:0] Y;
    wire [7:0] XC;
    wire [6:0] YC;
    wire [K-1:0] slow;
    wire go, sync;
    reg Ly, Ey, Lxc, Lyc, Exc, Eyc;
    wire Ydir;
    reg Tdir;
    reg [2:0] y_Q, Y_D;
	
	assign colour = SW[2:0];

    UpDn_count U1 (Y0, CLOCK_50, KEY[0], Ey, ~KEY[1], Ydir, Y);
        defparam U1.n = 7;

    regn U2 (X0, KEY[0], ~KEY[1], CLOCK_50, X);
        defparam U2.n = 8;

    UpDn_count U3 (8'd0, CLOCK_50, KEY[0], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
    UpDn_count U4 (7'd0, CLOCK_50, KEY[0], Eyc, Lyc, 1'b1, YC);
        defparam U4.n = 7;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, KEY[0], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);

    ToggleFF U6 (Tdir, KEY[0], CLOCK_50, Ydir);

    // FSM state table
    always @ (*)
        case (y_Q)
            A:  if (!go || !sync) Y_D = A;
                else Y_D = B;
            B:  if (XC != XDIM-1) Y_D = B;    // draw
                else Y_D = C;
            C:  if (YC != YDIM-1) Y_D = B;
                else Y_D = D;
            D:  if (!sync) Y_D = D;
                else Y_D = E;
            E:  if (XC != XDIM-1) Y_D = E;    // erase
                else Y_D = F;
            F:  if (YC != YDIM-1) Y_D = E;
                else Y_D = G;
            G:  Y_D = H;
            H:  Y_D = B;
        endcase
    // FSM outputs
    always @ (*)
    begin
        // default assignments
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; VGA_COLOR = colour; plot = 1'b0;
        Ey = 1'b0; Tdir = 1'b0;
        case (y_Q)
            A:  begin Lxc = 1'b1; Lyc = 1'b1; end
            B:  begin Exc = 1'b1; plot = 1'b1; end   // color a pixel
            C:  begin Lxc = 1'b1; Eyc = 1'b1; end
            D:  Lyc = 1'b1;
            E:  begin Exc = 1'b1; VGA_COLOR = ALT; plot = 1'b1; end   // color a pixel
            F:  begin Lxc = 1'b1; Eyc = 1'b1; end
            G:  begin Lyc = 1'b1; Tdir = (Y == 7'd0) || (Y == YSCREEN-1); end
            H:  Ey = 1'b1;
        endcase
    end

    always @(posedge CLOCK_50)
        if (!KEY[0])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

    assign go = ~KEY[3];

    assign VGA_X = X + XC;
    assign VGA_Y = Y + YC;
    // connect to VGA controller
    vga_adapter VGA (
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(VGA_COLOR),
			.x(VGA_X),
			.y(VGA_Y),
			.plot(plot),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK_N(VGA_BLANK_N),
			.VGA_SYNC_N(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif"; 
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

module ToggleFF(T, Resetn, Clock, Q);
    input T, Resetn, Clock;
    output reg Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (T)
            Q <= ~Q;
endmodule

module UpDn_count (R, Clock, Resetn, E, L, UpDn, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Clock, Resetn, E, L, UpDn;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (L == 1)
            Q <= R;
        else if (E)
            if (UpDn == 1)
                Q <= Q + 1;
            else
                Q <= Q - 1;
endmodule

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule
