/*
*   This code draws a horizontal line on the screen and then moves the line up and down. The
*   line "bounces" off the top and bottom of the screen and reverses directions. To run the demo
*   first press/release SW[9] to reset the circuit. Then, press/release SW[8] to initialize
*   the (x,y) location of the line. The line color is determined by SW[2:0]. Finally, press 
*   KEY[3] to start the animation. 
*/
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, HEX0, HEX1, HEX2, HEX3, HEX4);
	
    parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011; 
    parameter E = 4'b0100, F = 4'b0101, G = 4'b0110, H = 4'b0111; 
    parameter drawed = 4'b1000, erased = 4'b1001;
    parameter BB = 4'b1010, CC = 4'b1011; 
	parameter shift = 4'b1100, endGameState=4'b1101; 
    parameter hitState = 4'b1110; 


    parameter XSCREEN = 160, YSCREEN = 120;
    //parameter XDIM = XSCREEN>>1, YDIM = 1;
    parameter XDIM = 10, YDIM = 10;

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
    output HEX0, HEX1, HEX2, HEX3, HEX4;

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

    wire [7:0] XApple;
    wire [6:0] YApple;

    assign XApple = 8'd30;
    assign YApple = 7'd30;
        
    wire [7:0] XCApple;
    wire [6:0] YCApple;
    reg LxcApple, LycApple, ExcApple, EycApple;
	 
    // added
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

    // always @(posedge SW[4] or posedge SW[5]) 
    //     begin
    //     if (SW[5]) 
    //         begin
    //         currentLength <= 4'b1; // Reset to zero when sw[5] is high
    //         end 
    //     else if (currentLength < 4'b1111) 
    //         begin
    //         currentLength <= currentLength + 1; // Increment on the posedge of sw[4]
    //         end
    // end

    wire [3:0] currentLength;
    assign currentLength = 6;
    reg [3:0] drawBodyCount; 
    wire [8 * maxLength * XDIM -1 :0] XSnakeLong;
    wire [7 * maxLength * YDIM -1 :0] YSnakeLong;

   // assign XSnakeLong = {X0, X1, X2, X3};
 //   assign YSnakeLong = {Y0, Y1, Y2, Y3};

    reg Eshift;

    // initial begin
    //     for (i = 0; i < maxLength; i = i + 1) begin
    //         XSnakeLong[i * 8 +: 8] = X0 - (i * XDIM); // X coordinates, evenly spaced
    //         YSnakeLong[i * 7 +: 7] = Y0;             // Same Y coordinate
    //     end
    // end

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
    reg [3:0] y_Q, Y_D;
	
	assign colour = SW[2:0];

    UpDn_count U1 (Y0, CLOCK_50, SW[9], Ey, ~SW[8], Ydir, Y); // Sw[9] reset Sw[8] negative load
        defparam U1.n = 7;

    UpDn_count U2 (X0, CLOCK_50, SW[9], Ex, ~SW[8], Xdir, X);
        defparam U2.n = 8;



    // UpDn_count U22 (Y1, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[20 : 14]); // Sw[9] reset Sw[8] load
    //     defparam U22.n = 7;

    // UpDn_count U33 (X1, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[23 : 16]);
    //     defparam U33.n = 8;

    // UpDn_count U44 (Y2, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[13 : 7]); // Sw[9] reset Sw[8] load
    //     defparam U44.n = 7;

    // UpDn_count U55 (X2, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[15 : 8]);
    //     defparam U55.n = 8;

    // UpDn_count U66 (Y3, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[6:0]); // Sw[9] reset Sw[8] load
    //     defparam U66.n = 7;

    // UpDn_count U77 (X3, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[7:0]);
    //     defparam U77.n = 8;




    UpDn_count U3 (8'd0, CLOCK_50, SW[9], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
    UpDn_count U4 (7'd0, CLOCK_50, SW[9], Eyc, Lyc, 1'b1, YC);
        defparam U4.n = 7;

    UpDn_count U6 (8'd0, CLOCK_50, SW[9], ExcApple, LxcApple, 1'b1, XCApple);
        defparam U6.n = 8;
    UpDn_count U7 (7'd0, CLOCK_50, SW[9], EycApple, LycApple, 1'b1, YCApple);
        defparam U7.n = 7;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, SW[9], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);


    // UpDn_count U6 (maxLength, CLOCK_50, SW[9], Ebodycounter, ~SW[8], 1'b0, drawBodyCount);
    //     defparam U6.n = 2;


    hex7seg D0 (y_Q[0], HEX0);
    hex7seg D1 (y_Q[1], HEX1);
    hex7seg D2 (y_Q[2], HEX2);
    hex7seg D3 (y_Q[3], HEX3);
    // hex7seg D4 (y_Q[4], HEX4);

    // movement
    always @ (*)
    begin
    if (SW[5]) 
        begin
            move_right = 1'b0; move_down = 1'b0; move_up = 1'b0; move_left = 1'b0;
        end
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
                else Y_D = BB;

            BB:  if (XCApple != XDIM-1) Y_D = BB;    // draw apple
                else Y_D = CC;
            CC:  if (YCApple != YDIM-1) Y_D = BB;
                else Y_D = B;


            B:  if (XC != XDIM-1) Y_D = B;    // draw
                else Y_D = C;
            C:  if (YC != YDIM-1) Y_D = B;
                else Y_D = drawed;
            drawed: if (drawBodyCount > 1) Y_D = B;
                    else Y_D = D;
			D:  if (!sync) Y_D = D;
                else Y_D = E;
            E:  if (XC != XDIM-1) Y_D = E;    // erase
                else Y_D = F;
            F:  if (YC != YDIM-1) Y_D = E;
                else Y_D = erased; 
            erased: if (drawBodyCount > 1) Y_D = E;
                    else Y_D = hitState;
            hitState: Y_D = G;
            G:  begin if (!gameEnded)Y_D = H;	else Y_D = endGameState; end
            H:  Y_D = shift; // move
			shift:  Y_D = BB; // shiftreg
			endGameState: Y_D = endGameState; 
        endcase


    // // FSM outputsovement: load Y
    always @ (*)
    begin
        // default assignments
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; VGA_COLOR = colour; plot = 1'b0;
        Ex = 1'b0; Ey = 1'b0; Tdir_Y = 1'b0; Tdir_X = 1'b0;
    	ExcApple = 1'b0; EycApple = 1'b0;
        LxcApple = 1'b0; LycApple = 1'b0;
		Eshift = 1'b0;
        hitEnable = 1'b0;

        case (y_Q)
            A:  begin 
                Lxc = 1'b1; 
                Lyc = 1'b1; 
                LxcApple = 1'b1; 
                LycApple = 1'b1;
                end

            BB:  begin 
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
                // gameEnded = (Y == 7'd0) || (Y == YSCREEN- YDIM)||(X == 8'd0) || (X == XSCREEN- XDIM) || hit;
                //gameEnded = hit;

                end

            H:  
            begin
                LycApple = 1'b1; 
						 
         //   if (drawBodyCount == 1)
                //begin

                if (move_left)
                    begin
                    Ex <= 1'b1;
                    Xdir = 1'b0;
                    end
                else if (move_up)
                    begin
                    Ey <= 1'b1;
                    Ydir = 1'b0;
                    end
                else if (move_down)
                    begin
                    Ey <= 1'b1;
                    Ydir = 1'b1;
                    end
                else if (move_right)
                    begin
                    Ex <= 1'b1;
                    Xdir = 1'b1;
                    end
                end
					 
            shift: Eshift = 1'b1;
                    
            endGameState: VGA_COLOR=3'b111; 
					

          //  end
        endcase
    end

    always @(posedge CLOCK_50)
        if (!SW[9])
				begin
            y_Q <= 1'b0;
            drawBodyCount <= 6;
				end
        else
            begin
            y_Q <= Y_D;
				
            if ((y_Q == drawed && Y_D == B) || (y_Q == erased && Y_D == E))
                begin
            // if (drawBodyCount >= 1)  
                drawBodyCount <= drawBodyCount - 1;
                end
					 
					
            else if ( (y_Q == drawed && Y_D == D) || (y_Q == erased && Y_D == hitState))
                drawBodyCount <= 6;
                
            end

    assign go = SW[7];


    reg [7:0] VGA_X_reg, VGA_Y_reg;

    always @(*) begin
        VGA_X_reg = XSnakeLong[8 * XDIM * (maxLength - drawBodyCount + 1) -1 -: 8] + XC;
        VGA_Y_reg = YSnakeLong[7 * YDIM * (maxLength - drawBodyCount + 1) -1 -: 7] + YC;
        // VGA_X_reg = XSnakeLong[8 * drawBodyCount - 1 : 8 * drawBodyCount - 1 - 8] + XC;  // Dynamic part-select
        // VGA_Y_reg = YSnakeLong[7 * drawBodyCount - 1 : 7 * drawBodyCount - 1 - 7] + YC;  // Dynamic part-select
    end

    // assign VGA_X = VGA_X_reg;
    // assign VGA_Y = VGA_Y_reg;

    assign VGA_X = (y_Q == BB) ? (XApple + XCApple) : VGA_X_reg;
    assign VGA_Y = (y_Q == BB) ? (YApple + YCApple) : VGA_Y_reg;
    // assign VGA_X = XSnakeLong[8 * drawBodyCount - 1 : 8 * drawBodyCount - 1 - 8] + XC;
    // assign VGA_Y = YSnakeLong[7 * drawBodyCount - 1 : 7 * drawBodyCount - 1 - 7] + YC;

    // assign VGA_X = X + XC;
    // assign VGA_Y = Y + YC;

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
			  //data_out <= 0;
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
            hit = 1'b0; // Default: no collision
            if (currentLength >= 2)
            begin

            // Loop through all segments of the active snake length
            for (i = 4; i < maxLength; i = i + 1) begin
                if (i <= currentLength)
                begin
                // Check for collisions with each body segment
                if ((Xhead < XSnakeLong[(maxLength - i) * 8 * DIM - 1 -: 8] + DIM) && 
                    (Xhead + DIM > XSnakeLong[(maxLength - i) * 8 * DIM - 1 -: 8]) &&
                    (Yhead < YSnakeLong[(maxLength - i) * 7 * DIM - 1 -: 7] + DIM) && 
                    (Yhead + DIM > YSnakeLong[(maxLength - i) * 7 * DIM - 1 -: 7])) begin
                    hit = 1'b1; // Collision detected
                end
                end
            end

            end
        end
    end 
endmodule 




// module ifhit (enable, Xhead, Yhead, XSnakeLong, YSnakeLong, currentLength, hit, move_left, move_up, move_down, move_right);
//     parameter maxLength = 6;
//     parameter DIM = 10;
//     input enable;
//     input [7:0] Xhead;
// 	input [6:0] Yhead;
//     input [8 * maxLength * DIM -1 :0] XSnakeLong;
//     input [7 * maxLength * DIM -1 :0] YSnakeLong;
//     input [3:0] currentLength;
//     input move_left, move_up, move_down, move_right;
//     output reg hit;

//     integer i;

//     always @* begin
//         hit = 1'b0; // Default: no collision
//         if (enable && currentLength >= 2) begin
//             for (i = 2; i <= currentLength; i = i + 1) begin
//                 // Extract segment coordinates
//                 wire [7:0] x_segment = XSnakeLong[(maxLength - i) * 8 - 1 -: 8];
//                 wire [6:0] y_segment = YSnakeLong[(maxLength - i) * 7 - 1 -: 7];

//                 if (move_up)
//                 begin
//                     if ((Yhead == y_segment + DIM) &&
//                         (Xhead < x_segment + DIM) &&
//                         (Xhead + DIM > x_segment))
//                         hit = 1'b1;
//                 end
//                 if (move_down)
//                 begin
//                     if ((Yhead + DIM == y_segment) &&
//                         (Xhead < x_segment + DIM) &&
//                         (Xhead + DIM > x_segment))
//                         hit = 1'b1;
//                 end
//                 if (move_left)
//                 begin
//                     if ((Xhead == x_segment + DIM) &&
//                         (Yhead < y_segment + DIM) &&
//                         (Yhead + DIM > y_segment))
//                         hit = 1'b1;
//                 end
//                 if (move_right)
//                 begin
//                     if ((Xhead + DIM == x_segment) &&
//                         (Yhead < y_segment + DIM) &&
//                         (Yhead + DIM > y_segment))
//                         hit = 1'b1;
//                 end


//             end
//         end
//     end
// endmodule



// module ifhit (
//     enable,
//     Xhead,
//     Yhead,
//     XSnakeLong,
//     YSnakeLong,
//     currentLength,
//     hit, 
//     move_left, move_up, move_down, move_right
// );
//     parameter maxLength = 6;
//     parameter DIM = 10;
//     input enable;
//     input [7:0] Xhead;
//     input [6:0] Yhead;
//     input [8 * maxLength * DIM - 1 : 0] XSnakeLong;
//     input [7 * maxLength * DIM - 1 : 0] YSnakeLong;
//     input [3:0] currentLength;
//     input move_left, move_up, move_down, move_right; // 00 = right, 01 = left, 10 = up, 11 = down
//     output reg hit;

//     integer i;

//     always @* begin
//         if (enable) begin
//             hit = 1'b0; // Default: no collision
//             if (currentLength >= 2) begin
//                 // Loop through all segments of the active snake length
//                 for (i = 3; i < maxLength; i = i + 1) begin
//                     if (i <= currentLength) begin
//                         // Extract the current body segment position
//                         reg [7:0] XBody;
//                         reg [6:0] YBody;
//                         XBody = XSnakeLong[(maxLength - i) * 8 * DIM - 1 -: 8];
//                         YBody = YSnakeLong[(maxLength - i) * 7 * DIM - 1 -: 7];
                        
//                         // Check collision based on direction

//                 if (move_up)
//                 begin
//                     if ((Yhead == YBody + DIM) &&
//                         (Xhead < XBody + DIM) && (Xhead + DIM > XBody))
//                         hit = 1'b1;
//                 end
//                 if (move_down)
//                 begin
//                     if ((Yhead + DIM == YBody) &&
//                         (Xhead < XBody + DIM) && (Xhead + DIM > XBody))
//                         hit = 1'b1;
//                 end
//                 if (move_left)
//                 begin
//                     if ((Xhead == XBody + DIM) &&
//                         (Yhead < YBody + DIM) && (Yhead + DIM > YBody))
//                         hit = 1'b1;
//                 end
//                 if (move_right)
//                 begin
//                     if ((Xhead + DIM == XBody) &&
//                         (Yhead < YBody + DIM) && (Yhead + DIM > YBody))
//                         hit = 1'b1;
//                 end
    
//                     end
//                 end
//             end
//         end
//     end
// endmodule
