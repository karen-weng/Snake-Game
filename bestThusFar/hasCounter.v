/*
OUR SNAKE GAME!
karen & tyra
*/
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5);
	
    parameter A = 5'b00000, B = 5'b00001, C = 5'b00010, D = 5'b00011; 
    parameter E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111; 
    parameter drawed = 5'b01000, erased = 5'b01001;
    parameter BB = 5'b01010, CC = 5'b01011; 
	parameter shift = 5'b01100, endGameState=5'b01101; 
    parameter hitState = 5'b01110; 
    parameter waitKey = 5'b01111; 
    parameter Binital = 5'b10000, Cinital = 5'b10001, drawedInital = 5'b10010;
    parameter EE = 5'b10011, FF = 5'b10100;
    parameter BBinital = 5'b10101, CCinital = 5'b10110; 


    parameter XSCREEN = 160, YSCREEN = 120;
    //parameter XDIM = XSCREEN>>1, YDIM = 1;
    parameter XDIM = 10, YDIM = 10, YDimApp=4, XDimApp=4;

    parameter X0 = 8'd80, Y0 = 7'd60;
    parameter X1 = 8'd80, Y1 = 7'd70;
    parameter X2 = 8'd80, Y2 = 7'd80;
    parameter X3 = 8'd80, Y3 = 7'd90;
    parameter X4 = 8'd80, Y4 = 7'd100;
    parameter X5 = 8'd80, Y5 = 7'd110;
    parameter ALT = 3'b000; // alternate object color
    parameter K = 20; // animation speed: use 20 for hardware, 2 for ModelSim

    input CLOCK_50;	
    input [9:0] SW;
    input [3:0] KEY;
    output [7:0] VGA_R;
    output [7:0] VGA_G;
    output [7:0] VGA_B;
    output VGA_HS;
    output VGA_VS;
    output VGA_BLANK_N;
    output VGA_SYNC_N;
    output VGA_CLK;	
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    wire [7:0] VGA_X; 
    wire [6:0] VGA_Y;  
    reg [2:0] VGA_COLOR;
    reg plot;

    wire [2:0] colour;
    wire [7:0] X;
    wire [6:0] Y;
    wire [7:0] XC;
    wire [6:0] YC;
    wire [K-1:0] slow;
    wire go, sync;
    reg Ex, Ey, Lxc, Lyc, Exc, Eyc;

    reg [7:0] XApple;
    reg [6:0] YApple;
    reg eatApple;
	reg[1:0]found;

    wire [7:0] XCApple;
    wire [6:0] YCApple;
    reg LxcApple, LycApple, ExcApple, EycApple;
	 
	reg Xdir;
    reg Ydir;
	 
	reg gameEnded;

    reg move_left, move_up, move_down, move_right;

    parameter maxLength = 6;
    // wire maxLength;
    // assign maxLength = 2;

    // reg [3:0] currentLength;
    wire hit;
    reg hitEnable;

    wire [3:0] currentLength;
    assign currentLength = 6;
    reg [3:0] drawBodyCount; 
    wire [8 * maxLength * XDIM -1 :0] XSnakeLong;
    wire [7 * maxLength * YDIM -1 :0] YSnakeLong;
	reg [3:0] counter; 

    reg Eshift;

    shift_register_move_snake S0 (CLOCK_50, Eshift, SW[5], XSnakeLong, X, XSnakeLong);
        defparam S0.n = 8; 
		defparam S0.P0 = X0;
        defparam S0.P1 = X1;
        defparam S0.P2 = X2;
        defparam S0.P3 = X3;
        defparam S0.P4 = X4;
        defparam S0.P5 = X5;
		  
		  
    shift_register_move_snake S1 (CLOCK_50, Eshift, SW[5], YSnakeLong, Y, YSnakeLong);
        defparam S1.n = 7; 
		defparam S1.P0 = Y0;
        defparam S1.P1 = Y1;
        defparam S1.P2 = Y2;
        defparam S1.P3 = Y3;
        defparam S1.P4 = Y4;
        defparam S1.P5 = Y5;

    ifhit H1 (hitEnable, X, Y, XSnakeLong, YSnakeLong, currentLength, hit, move_left, move_up, move_down, move_right);

    reg Tdir_X;
    reg Tdir_Y;
    reg [4:0] y_Q, Y_D;
	
	assign colour = SW[2:0];

    UpDn_count U1 (Y0, CLOCK_50, SW[9], Ey, ~SW[8], Ydir, Y); // Sw[9] reset Sw[8] negative load
        defparam U1.n = 7;

    UpDn_count U2 (X0, CLOCK_50, SW[9], Ex, ~SW[8], Xdir, X);
        defparam U2.n = 8;

    UpDn_count U3 (8'd0, CLOCK_50, SW[9], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
    UpDn_count U4 (7'd0, CLOCK_50, SW[9], Eyc, Lyc, 1'b1, YC);
        defparam U4.n = 7;

    UpDn_count U6 (8'd0, CLOCK_50, SW[9], ExcApple, LxcApple, 1'b1, XCApple);
        defparam U6.n = 4;
    UpDn_count U7 (7'd0, CLOCK_50, SW[9], EycApple, LycApple, 1'b1, YCApple);
        defparam U7.n = 4;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, SW[9], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);

    // hex7seg D0 (y_Q[0], HEX1);
    // hex7seg D1 (y_Q[1], HEX2);
    // hex7seg D2 (y_Q[2], HEX3);
    // hex7seg D3 (y_Q[3], HEX4);
    // hex7seg D4 (y_Q[4], HEX5);

    decimal_display a1(counter[3:0], HEX0); 

    // movement
    always @ (*)
    begin
        // Direction control based on key inputs
    if (~KEY[0]) // Move Right
        begin
            move_right = 1'b1; move_down = 1'b0; move_up = 1'b0; move_left = 1'b0;
        end
    else if (~KEY[1]) // Move Down
        begin
            move_right = 1'b0; move_down = 1'b1; move_up = 1'b0; move_left = 1'b0;
        end
    else if (~KEY[2]) // Move Up
        begin
            move_right = 1'b0; move_down = 1'b0; move_up = 1'b1; move_left = 1'b0;
        end
    else if (~KEY[3]) // Move Left
        begin
            move_right = 1'b0; move_down = 1'b0; move_up = 1'b0; move_left = 1'b1;
        end
    end



    // FSM state table
    always @ (*)
        case (y_Q)
            A:  if (!go || !sync) Y_D = A;
                else Y_D = BBinital;
					
            BBinital:  if (XCApple != XDimApp-1) Y_D = BBinital;    // draw apple
                else Y_D = CCinital;
            CCinital:  if (YCApple != YDimApp-1) Y_D = BBinital;
                else Y_D = Binital;

            Binital:  if (XC != XDIM-1) Y_D = Binital;    // draw snake inital
                else Y_D = Cinital;
            Cinital:  if (YC != YDIM-1) Y_D = Binital;
                else Y_D = drawedInital;

            drawedInital: Y_D = waitKey;

            waitKey: if (~KEY[0] || ~KEY[1] || ~KEY[2] || ~KEY[3]) Y_D = BB;
                    else Y_D = waitKey;

            BB:  if (XCApple != XDimApp-1) Y_D = BB;    // draw apple
                else Y_D = CC;
            CC:  if (YCApple != YDimApp-1) Y_D = BB;
                else Y_D = B;

            B:  if (XC != XDIM-1) Y_D = B;    // draw snake
                else Y_D = C;
            C:  if (YC != YDIM-1) Y_D = B;
                else Y_D = drawed;

            drawed: if (drawBodyCount > 1) Y_D = B; // loads
                    else Y_D = D;

			D:  if (!sync) Y_D = D;
                else Y_D = E;
            E:  if (XC != XDIM-1) Y_D = E;    // erase
                else Y_D = F;
            F:  if (YC != YDIM-1) Y_D = E;
                else Y_D = erased; 
            erased: if (drawBodyCount > 1) Y_D = E; // loads
                    else Y_D = hitState;

            hitState: Y_D = G;
            G:  if (!gameEnded)Y_D = H;	
                else Y_D = endGameState;

            H:  if (!eatApple) Y_D = shift;
			    else Y_D = EE;	// move

			EE: if (XCApple != XDimApp-1) Y_D = EE;   
                else Y_D = FF;
            FF: if (YCApple != YDimApp-1) Y_D = EE;
                else Y_D = shift;

            shift:  Y_D = BB; // shiftreg

            endGameState: Y_D = endGameState; 
        endcase


    // // FSM outputsovement: load Y
    always @ (*)
    begin
        // default assignments
        VGA_COLOR = colour; plot = 1'b0;
        Lxc = 1'b0; Lyc = 1'b0; 
        Exc = 1'b0; Eyc = 1'b0;  
        LxcApple = 1'b0; LycApple = 1'b0;
    	ExcApple = 1'b0; EycApple = 1'b0;
        Ex = 1'b0; Ey = 1'b0; 

		Eshift = 1'b0;
        hitEnable = 1'b0;
        gameEnded = 1'b0;

        case (y_Q)
            A:  begin 
                Lxc = 1'b1; 
                Lyc = 1'b1; 
                LxcApple = 1'b1; 
                LycApple = 1'b1;
				XApple = 8'd30;
				YApple = 7'd30;
                counter = 1;
                end

            BBinital:  
                begin 
				ExcApple = 1'b1; 
				VGA_COLOR = 3'b100; 
                plot = 1'b1; 
				end // color a pixel

            CCinital:  
                begin 
				LxcApple = 1'b1; 
				EycApple = 1'b1; 
				end

            Binital:  begin Exc = 1'b1; plot = 1'b1; end   // color a pixel

            Cinital:  begin Lxc = 1'b1; Eyc = 1'b1; end

            drawedInital: Lyc = 1'b1;

            waitKey: 
                begin 
                Lxc = 1'b1; 
                Lyc = 1'b1; 
                LxcApple = 1'b1; 
                LycApple = 1'b1;
                end

            BB: begin 
				ExcApple = 1'b1; 
				VGA_COLOR = 3'b100; 
                plot = 1'b1; 
				end // color a pixel

            CC:  begin 
				LxcApple = 1'b1; 
				EycApple = 1'b1; 
				end

            B:  begin Exc = 1'b1; plot = 1'b1; end   // color a pixel

            C:  begin Lxc = 1'b1; Eyc = 1'b1; end

            drawed: Lyc = 1'b1;
           // D:  
            E:  begin 
                Exc = 1'b1; 
                VGA_COLOR = ALT; 
                plot = 1'b1; 
                end   // color a pixel

            F:  begin 
                Lxc = 1'b1; 
                Eyc = 1'b1; 
                end

            erased: Lyc = 1'b1; 
            hitState: hitEnable = 1'b1;
            G:   begin 
                gameEnded = (Y == 7'd0) || (Y == YSCREEN- YDIM)||(X == 8'd0) || (X == XSCREEN- XDIM) || hit;
                // gameEnded = hit;
                end

            H:  
            begin
                LycApple = 1'b1; 

                if (move_left)
                    begin
                    Ex = 1'b1;
                    Xdir = 1'b0;
                    end
                else if (move_up)
                    begin
                    Ey = 1'b1;
                    Ydir = 1'b0;
                    end
                else if (move_down)
                    begin
                    Ey = 1'b1;
                    Ydir = 1'b1;
                    end
                else if (move_right)
                    begin
                    Ex = 1'b1;
                    Xdir = 1'b1;
                    end
                	
                eatApple = ((X < XApple + XDimApp) && (X + XDIM > XApple)) && ((Y < YApple + YDimApp) && (Y + YDIM > YApple));
				if (eatApple)
					begin
					found=2'b11; 
					eatApple=1'b0;
					if (!gameEnded) counter <= counter + 1;
					end	
					
				if (found==2'b11)
				    begin
					case(XApple)
					8'd30: XApple=8'd10; 
					8'd10: XApple=8'd70;
					8'd70: XApple=8'd60;
					8'd90: XApple=8'd20;
					8'd20: XApple=8'd50;
					8'd50: XApple=8'd80;
					default: XApple=8'd10; 
					endcase
					case(YApple)
					7'd30: YApple=7'd20; 
					7'd20: YApple=7'd90;
					7'd90: YApple=7'd100;
					7'd100: YApple=7'd80;
					7'd80: YApple=7'd10;
					7'd10: YApple=7'd50;
					default: YApple=7'd20; 
					endcase
					found=2'b10; 
				    end

				else if (found==2'b00)
					begin
					XApple=8'd30; 
					YApple=7'd30; 
					found=2'b10; 
					end
                end

            shift: Eshift = 1'b1;
            endGameState: 
                begin 
                    plot =1'b1; 
                    VGA_COLOR=3'b101; 
                    Exc=1'b1; 
                    Eyc=(XC==XDIM-1);
                
                end
        endcase
    end

    always @(posedge CLOCK_50)
        if (!SW[9])
				begin
            y_Q <= 1'b0;
            drawBodyCount <= currentLength;
				end
        else
            begin
            y_Q <= Y_D;
				
            if ((y_Q == drawed && Y_D == B) || (y_Q == erased && Y_D == E))
                drawBodyCount <= drawBodyCount - 1;
					
            else if ( (y_Q == drawed && Y_D == D) || (y_Q == erased && Y_D == hitState))
                drawBodyCount <= currentLength;
                
            end

    assign go = SW[7];


    // displaying on vga
    reg [7:0] VGA_X_reg, VGA_Y_reg;

    always @(*) begin
        if (y_Q == Binital)
            begin 
            VGA_X_reg = X0 + XC;
            VGA_Y_reg = Y0 + YC;
            end
        else 
            begin
            VGA_X_reg = XSnakeLong[8 * XDIM * (maxLength - drawBodyCount + 1) -1 -: 8] + XC;
            VGA_Y_reg = YSnakeLong[7 * YDIM * (maxLength - drawBodyCount + 1) -1 -: 7] + YC;
            end
    end

    assign VGA_X = (y_Q == BB || y_Q == BBinital) ? (XApple + XCApple) : VGA_X_reg;
    assign VGA_Y = (y_Q == BB || y_Q == BBinital) ? (YApple + YCApple) : VGA_Y_reg;

    // connect to VGA controller
    vga_adapter VGA (
			.resetn(SW[9]),
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

module shift_register_move_snake (clk, enable, reset, data, data_in, data_out);    
	parameter n = 8;
    parameter maxLength = 6;
    parameter DIM = 10;
	 
    input clk;
    input enable, reset;
    // input maxLength; 
    input [ n * maxLength * DIM - 1 : 0 ] data;
    input [ n - 1 : 0 ] data_in;
    output reg [ n * maxLength * DIM - 1 : 0 ] data_out;

    parameter [n - 1 : 0] P0 = {n{1'b0}}, 
                      P1 = {n{1'b0}}, 
                      P2 = {n{1'b0}}, 
                      P3 = {n{1'b0}},
                      P4 = {n{1'b0}},
                      P5 = {n{1'b0}};

    always @(posedge clk) 
    begin
        if (reset) begin
           data_out <= {{DIM{P0}}, {DIM{P1}}, {DIM{P2}}, {DIM{P3}}, {DIM{P4}}, {DIM{P5}}};
        end
        
        if (enable) begin
            // left is the head
            // add new data to the front the rest follows
            data_out <= {data_in, data[n * maxLength * DIM -1 : n ]};
        end
    end

endmodule

module ifhit (enable, Xhead, Yhead, XSnakeLong, YSnakeLong, currentLength, hit, move_left, move_up, move_down, move_right);
    parameter maxLength = 6;
    parameter DIM = 10;
    input enable;
    input [7:0] Xhead;
	input [6:0] Yhead;
    input [8 * maxLength * DIM -1 :0] XSnakeLong;
    input [7 * maxLength * DIM -1 :0] YSnakeLong;
    input [3:0] currentLength;
    input move_left, move_up, move_down, move_right;
    output reg hit;

    integer i;

    always @* begin
        if (enable)
        begin
            hit = 1'b0; 

            for (i = 2; i < maxLength; i = i + 1) begin
                if (i <= currentLength)
                begin
                if ((Xhead < XSnakeLong[(maxLength - i) * 8 * DIM - 1 -: 8] + DIM) && 
                    (Xhead + DIM > XSnakeLong[(maxLength - i) * 8 * DIM - 1 -: 8]) &&
                    (Yhead < YSnakeLong[(maxLength - i) * 7 * DIM - 1 -: 7] + DIM) && 
                    (Yhead + DIM > YSnakeLong[(maxLength - i) * 7 * DIM - 1 -: 7])) 
                    begin
                    hit = 1'b1; 
                    end
                end
            end
        end
    end 
endmodule 

module decimal_display (C, Display); 
    input [3:0] C; 
    output [6:0] Display; 

    assign Display[0] = ~C[3] & ~C[2] & ~C[1] & C[0] | ~C[3] & C[2] & ~C[1] & ~C[0];
    assign Display[1] = ~C[3] & C[2] & ~C[1] & C[0] | ~C[3] & C[2] & C[1] & ~C[0];
    assign Display[2] = ~C[3] & ~C[2] & C[1] & ~C[0];
    assign Display[3] = ~C[2] & ~C[1] & C[0] | ~C[3] & C[2] & ~C[1] & ~C[0] | ~C[3] & C[2] & C[1] & C[0];
    assign Display[4] = ~C[1] & C[2] | ~C[2] & C[0] | ~C[3] & C[2] & C[0]; 
    assign Display[5] = ~C[2] & C[1] | C[1] & C[0] | ~C[3] & ~C[2] & ~C[1] & C[0]; 
    assign Display[6] = ~C[3] & ~C[2] & ~C[1] | C[2] & C[1] & C[0];
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
