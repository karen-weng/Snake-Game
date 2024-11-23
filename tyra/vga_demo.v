
/*
*   This code draws a horizontal line on the screen and then moves the line up and down. The
*   line "bounces" off the top and bottom of the screen and reverses directions. To run the demo
*   first press/release SW[9] to reset the circuit. Then, press/release SW[8] to initialize
*   the (x,y) location of the line. The line color is determined by SW[2:0]. Finally, press 
*   KEY[3] to start the animation. 
*/
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, HEX0, HEX1);
	
    parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011; 
    parameter E = 4'b0100, F = 4'b0101, G = 4'b0110, H = 4'b0111; 
    parameter BB = 4'b1000, CC = 4'b1001, EndGame = 4'b1010, EE=4'b1011, FF=4'b1100; 
    parameter XSCREEN = 160, YSCREEN = 120;
    //parameter XDIM = XSCREEN>>1, YDIM = 1;
    parameter XDIM = 10, YDIM = 10;
	parameter XDimApp=4, YDimApp=4; 
    parameter XApple0 = 8'd30;
    parameter YApple0 = 7'd30;
        
    parameter X0 = 8'd80, Y0 = 7'd60;//changed to 80 and 60 so it starts inthe middle
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
	output [6:0] HEX0, HEX1; 
    wire [7:0] VGA_X; 
    wire [6:0] VGA_Y;  
    reg [2:0] VGA_COLOR;
    reg plot;
        
    reg [7:0] Xdraw, XCdraw;
    reg [6:0] Ydraw, YCdraw;
	 
    wire [2:0] colour;
	 assign colour=3'b110;
	// assign VGA_COLOUR=3'b110; 
    wire [7:0] X;
    wire [6:0] Y;
	 reg [7:0] Xtrack; 
	reg [6:0] Ytrack; 
    wire [7:0] XC;
    wire [6:0] YC;
    wire [K-1:0] slow;
    wire go, sync;
    reg Ex, Ey, Lxc, Lyc, Exc, Eyc, endgame, eatApple;
	 reg[1:0]found;
    reg[4:0] counter; 
    reg [7:0] XApple;//changed from wire to reg
    reg [6:0] YApple;
//	 always @ (posedge SW[8])
//		XApple=8'b80
	
   // assign XApple = 8'd30;
   // assign YApple = 7'd30;
    
    wire [7:0] XCApple;
    wire [6:0] YCApple;
    reg LxcApple, LycApple, ExcApple, EycApple;
        
    // added
    reg Xdir;
    reg Ydir;
	// reg commanded;
	// reg colourSnake;

    reg move_left, move_up, move_down, move_right;


    reg Tdir_X;
    reg Tdir_Y;
    reg [3:0] y_Q, Y_D;
	 
	 decimal_display a1(counter[3:0], HEX0); 
	// decimal_display a2 (counter [7:4], HEX1); 
//	 decimal_display a3(counter[11:8], HEX2); 
//	 decimal_display a4 (counter [15:12], HEX3); 
    UpDn_count U1 (Y0, CLOCK_50, SW[9], Ey, ~SW[8], Ydir, Y); // Sw[9] reset Sw[8] load
        defparam U1.n = 7;

    UpDn_count U2 (X0, CLOCK_50, SW[9], Ex, ~SW[8], Xdir, X);
        defparam U2.n = 8;
		  
    // UpDn_count U8 (YApple0, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YApple); // Sw[9] reset Sw[8] load
    //     defparam U8.n = 7;

    // UpDn_count U9 (XApple0, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XApple);
    //     defparam U9.n = 8;


    UpDn_count U3 (8'd0, CLOCK_50, SW[9], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
    UpDn_count U4 (7'd0, CLOCK_50, SW[9], Eyc, Lyc, 1'b1, YC);
        defparam U4.n = 7;
            
//    UpDn_count U6 (8'd0, CLOCK_50, SW[9], ExcApple, LxcApple, 1'b1, XCApple);
//        defparam U6.n = 8;
//    UpDn_count U7 (7'd0, CLOCK_50, SW[9], EycApple, LycApple, 1'b1, YCApple);
//        defparam U7.n = 7;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, SW[9], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);
		//assign VGA_COLOR=colour;
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
            // A:  if (!go || !sync) Y_D = A;
            A:  if (SW[7] || !sync) Y_D = A;

                else Y_D = BB;

            BB:  if (XCApple != XDimApp-1) Y_D = BB;    // draw apple
                else Y_D = CC;
            CC:  if (YCApple != YDimApp-1) Y_D = BB;
                else Y_D = B;

            B:  if (XC != XDIM-1) Y_D = B;    // draw
                else Y_D = C;
            C:  if (YC != YDIM-1) Y_D = B;
                else Y_D = D;
            D:  if (!sync) Y_D = D;
                else Y_D = E;

            // DD:  if (!sync) Y_D = DD;
            //     else Y_D = E;
            E:  if (XC != XDIM-1) Y_D = E;    // erase
                else Y_D = F;
            F:  if (YC != YDIM-1) Y_D = E;
                else Y_D = G;
            G:  //Y_D = H; // edge detection
					if (endgame)
						Y_D=EndGame; 
//					else if (eatApple) Y_D=ate; 
					else Y_D=H; 
            H:  if(!eatApple)Y_D = BB;
					else Y_D=EE; // move
				EndGame: Y_D=EndGame;
				EE: if (XCApple != XDimApp-1) Y_D = EE;   
                else Y_D = FF;
            FF: if (YCApple != YDimApp-1) Y_D = EE;
                else Y_D = BB;
        endcase
	//always @(*)
	//begin
	//if (SW[9]) y_Q<=Y_D;
	//else y_Q<=4'b0; 
	//end
	reg start; 
    // // FSM outputsovement: load Y
    always @ (*)
    begin
        // default assignments 
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; VGA_COLOR = colour;
		  plot = 1'b0; eatApple=1'b0; 
        Ex = 1'b0; Ey = 1'b0; Tdir_Y = 1'b0; Tdir_X = 1'b0;
		ExcApple = 1'b0; EycApple = 1'b0;
        LxcApple = 1'b0; LycApple = 1'b0; 
		  Ytrack=Y; Xtrack=X; //endgame=0;
		 // object_mem b1 ({Y,X}, CLOCK_50, colour);
		  //if (endgame)begin commanded=1; VGA_COLOR=3'b111; end
		  //else begin commanded=0; end
		  //found=2'b00; caused us to always have y value of 30 30
			//whats the issue with the way im calling the decimal display
					
        case (y_Q)
            A:  begin 
                Lxc = 1'b1; 
                Lyc = 1'b1; 
                LxcApple = 1'b1; 
                LycApple = 1'b1;
					 XApple=8'd30;
					YApple=7'd30;
					// found=2'b0; 
                end

            BB:  begin 
						//if (!eatApple)begin
						ExcApple = 1'b1; 
						VGA_COLOR = 3'b100; 
						plot = 1'b1; 
					//	if (eatApple) begin found=2'b10; eatApple=1'b0; end
					
//						end
//						else begin
//						ExcApple = 1'b1; 
//						VGA_COLOR = 3'b000; 
//						plot = 1'b1; end
//						XApple=XCApple; 
//						YApple=YCApple; 
					//	eatApple=1'b0;
					 
				// Xdraw = XApple;
				// XCdraw = XCApple;
				// Ydraw = YApple;
				// YCdraw = YCApple;
				end // color a pixel
				
            CC:  begin 
				LxcApple = 1'b1; 
				EycApple = 1'b1; 
				// VGA_COLOR = 3'b100; 
				end

            B:  begin 
				Exc = 1'b1; 
				plot = 1'b1; 
				// Xdraw = X;
				// XCdraw = XC;
				// Ydraw = Y;
				// YCdraw = YC;
				end   // color a pixel
            C:  begin 
				Lxc = 1'b1; 
				Eyc = 1'b1; 
				
				end

            D:  Lyc = 1'b1;
            // Lxc = 1'b1; 
				
            // DD:  Lyc = 1'b1;
				
            E:  begin 
				Exc = 1'b1; 
				VGA_COLOR = ALT; 
				plot = 1'b1; 
				// Xdraw = X;
				// XCdraw = XC;
				// Ydraw = Y;
				// YCdraw = YC;
				end   // color a pixel

            F:  begin 
                Lxc = 1'b1; 
                Eyc = 1'b1; 
                end

				EE: begin 
				ExcApple = 1'b1; 
				VGA_COLOR = ALT; 
				plot = 1'b1; 
				// Xdraw = X;
				// XCdraw = XC;
				// Ydraw = Y;
				// YCdraw = YC;
				end   // color a pixel

            FF:  begin 
                LxcApple = 1'b1; 
                EycApple = 1'b1; 
                end
					 
					 
            G:  begin 
                Lyc = 1'b1; 
                LycApple = 1'b1; 
					 //colorChange=(Y==30);
					 	endgame= (Y == 7'd0) || (Y == YSCREEN- YDIM)|| (X == 8'd0) || (X == XSCREEN- XDIM);
						
                //Tdir_Y = (Y == 7'd0) || (Y == YSCREEN- YDIM);  // Flip Ydir at vertical edges
                //Tdir_X = (X == 8'd0) || (X == XSCREEN- XDIM);  // Flip Xdir at horizontal edges

                // Adjust Tdir_X and Tdir_Y based on the active direction flags
                // if (move_right)
                //     Tdir_X = (X < XSCREEN - XDIM) ? 1'b1 : 1'b0; // Move right, stop at screen edge
                // else if (move_left)
                //     Tdir_X = (X > 0) ? 1'b0 : 1'b1; // Move left, stop at screen edge
                // else
                //     Tdir_X = 1'b0; // Default to no horizontal movement

                // if (move_down)
                //     Tdir_Y = (Y < YSCREEN - YDIM) ? 1'b1 : 1'b0; // Move down, stop at screen edge
                // else if (move_up)
                //     Tdir_Y = (Y > 0) ? 1'b0 : 1'b1; // Move up, stop at screen edge
                // else
                //     Tdir_Y = 1'b0; // Default to no vertical movement
            end
				EndGame: begin plot =1'b1; VGA_COLOR=3'b101; Exc=1'b1; Eyc=(XC==XDIM-1);end

            H:  
            begin
            if (move_left)
				begin
                Ex <= 1'b1;
                Xdir = 1'b0;
					 Xtrack=Xtrack-1; 
					 //commanded=1;
				end
            else if (move_up)
				begin
                Ey <= 1'b1;
                Ydir = 1'b0;
					 Ytrack=Ytrack+1; 
					 //commanded=1;
				end
            else if (move_down)
				begin
                Ey <= 1'b1;
                Ydir = 1'b1;
					 Ytrack=Ytrack-1;
					 //commaned=1;
				end
            else if (move_right)
				begin
                Ex <= 1'b1;
                Xdir = 1'b1;
					  Xtrack=Xtrack+1;
					// commanded=1; 
                end
					 eatApple = ((X < XApple + XDIM) && (X + XDIM > XApple)) && ((Y < YApple + YDIM) && (Y + YDIM > YApple));
					//eatApple=((Y<YApple&&(Y+YDIM>YApple))||((Y<YApple+7'd4)&&(Y>YApple)))||((X<XApple&&(X+XDIM>XApple))||((X<XApple+8'd4)&&(X>XApple)));
				if (eatApple)
					begin
					found=2'b11; 
					eatApple=1'b0;
					//counter
					if (!endgame) counter<=counter+1;
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
					//eatApple=1'b0; 
			//counter<=counter+1; 
				end
				else if (found==2'b00)
					begin
					XApple=8'd30; 
					YApple=7'd30; 
					found=2'b10; 
					end	
				
				///endgame= (Y == 7'd0) || (Y == YSCREEN- YDIM)|| (X == 8'd0) || (X == XSCREEN- XDIM);
            end
				

        endcase
		 // if (commanded)
			//	VGA_COLOR=3'b101;/
		  
		  //took out condition commanded==1
		end
			//always @ (*) begin
   // if (!commanded) colour =3'b110;
	// else colour=3'b100; 
	// end
	 //VGA_COLOR=colour; 
//	 //where shoudl i put thisi
//	always @ (*)//cound==11 meqans found, found==10 means started not found and 00 means not started and needs to be initailzed
//		//why is it only printing on the top right corner
//		begin
//		if (eatApple)
//			found=2'b11; 
//		if (found==2'b11)
//			begin
//			case(XApple)
//			8'd30: XApple=8'd70;
//			8'd70: XApple=8'd60;
//			8'd90: XApple=8'd20;
//			endcase
//			case(YApple)
//			7'd30: YApple=8'd90;
//			7'd90: YApple=8'd30;
//			7'd20: YApple=8'd80;
//			endcase
//			found=2'b10; 
//			//counter<=counter+1; 
//			end
//		else if (found==2'b00)
//			begin
//			XApple=8'd30; 
//			YApple=7'd30; 
//			found=2'b10; 
//			end
//		end
//		
	UpDn_count U6 (8'b0, CLOCK_50, SW[9], ExcApple, LxcApple, 1'b1, XCApple);
        defparam U6.n = 4;
    UpDn_count U7 (7'b0, CLOCK_50, SW[9], EycApple, LycApple, 1'b1, YCApple);
        defparam U7.n = 4;	

    always @(posedge CLOCK_50)
        if (!SW[9])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

    assign go = ~SW[7];

    // assign VGA_X = Xdraw + XCdraw;
    // assign VGA_Y = Ydraw + YCdraw;

    assign VGA_X = (y_Q == BB) ? (XApple + XCApple) : (X + XC);
    assign VGA_Y = (y_Q == BB) ? (YApple + YCApple) : (Y + YC);
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