/*
*   This code draws a horizontal line on the screen and then moves the line up and down. The
*   line "bounces" off the top and bottom of the screen and reverses directions. To run the demo
*   first press/release SW[9] to reset the circuit. Then, press/release SW[8] to initialize
*   the (x,y) location of the line. The line color is determined by SW[2:0]. Finally, press 
*   KEY[3] to start the animation. 
*/
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
	
    parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011; 
    parameter E = 3'b100, F = 3'b101, G = 3'b110, H = 3'b111; 
    parameter XSCREEN = 160, YSCREEN = 120;
    //parameter XDIM = XSCREEN>>1, YDIM = 1;
    parameter XDIM = 10, YDIM = 10;

    parameter X0 = 8'd39, Y0 = 7'd59;
    parameter X1 = 8'd59, Y1 = 7'd59;
    parameter X2 = 8'd79, Y2 = 7'd59;
    parameter X3 = 8'd99, Y3 = 7'd59;
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

    wire [7:0] VGA_X; 
    wire [6:0] VGA_Y;  
    reg [2:0] VGA_COLOR;
    reg plot;

    wire [7:0] Xapple;
    wire [6:0] Yapple; 

    assign Xapple = 8'd80;
    assign Yapple = 7'd60; 


    wire [2:0] colour;
    wire [7:0] X;
    wire [6:0] Y;
    wire [7:0] XC;
    wire [6:0] YC;
    wire [K-1:0] slow;
    wire go, sync;
    reg Ex, Ey, Lxc, Lyc, Exc, Eyc;
	 
    // added
	reg Xdir;
    reg Ydir;

    reg move_left, move_up, move_down, move_right;

    parameter maxLength = 4;
    // wire maxLength;
    // assign maxLength = 2;

    reg drawBodyCount; 
    wire [8 * maxLength - 1:0] XSnakeLong;
    wire [7 * maxLength - 1:0] YSnakeLong;
    reg Ebodycounter;

    // initial begin
    //     for (i = 0; i < maxLength; i = i + 1) begin
    //         XSnakeLong[i * 8 +: 8] = X0 - (i * XDIM); // X coordinates, evenly spaced
    //         YSnakeLong[i * 7 +: 7] = Y0;             // Same Y coordinate
    //     end
    // end

    shift_register_move_snake S0 (CLOCK_50, SW[9], XSnakeLong, X, XSnakeLong);
        defparam S0.n = 8; 
    shift_register_move_snake S1 (CLOCK_50, SW[9], YSnakeLong, Y, YSnakeLong);
        defparam S0.n = 7; 

    reg Tdir_X;
    reg Tdir_Y;
    reg [2:0] y_Q, Y_D;
	
	assign colour = SW[2:0];

    UpDn_count U1 (Y0, CLOCK_50, SW[9], Ey, ~SW[8], Ydir, Y); // Sw[9] reset Sw[8] load
        defparam U1.n = 7;

    UpDn_count U2 (X0, CLOCK_50, SW[9], Ex, ~SW[8], Xdir, X);
        defparam U2.n = 8;



    UpDn_count U22 (Y1, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[7 * 3 - 1 : 7 * 3 - 1 - 7]); // Sw[9] reset Sw[8] load
        defparam U22.n = 7;

    UpDn_count U33 (X1, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[8 * 3 - 1 : 8 * 3 - 1 - 8]);
        defparam U33.n = 8;

    UpDn_count U44 (Y2, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[7 * 2 - 1 : 7 * 2 - 1 - 7]); // Sw[9] reset Sw[8] load
        defparam U44.n = 7;

    UpDn_count U55 (X2, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[8 * 2 - 1 : 8 * 2 - 1 - 8]);
        defparam U55.n = 8;

    UpDn_count U66 (Y3, CLOCK_50, SW[9], 1'b0, ~SW[8], Ydir, YSnakeLong[7 * 1 - 1 : 7 * 1 - 1 - 7]); // Sw[9] reset Sw[8] load
        defparam U66.n = 7;

    UpDn_count U77 (X3, CLOCK_50, SW[9], 1'b0, ~SW[8], Xdir, XSnakeLong[8 * 1 - 1 : 8 * 1 - 1 - 8]);
        defparam U77.n = 8;




    UpDn_count U3 (8'd0, CLOCK_50, SW[9], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
    UpDn_count U4 (7'd0, CLOCK_50, SW[9], Eyc, Lyc, 1'b1, YC);
        defparam U4.n = 7;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, SW[9], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);


    // UpDn_count U6 (maxLength, CLOCK_50, SW[9], Ebodycounter, ~SW[8], 1'b0, drawBodyCount);
    //     defparam U6.n = 2;

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
            G:  Y_D = H; // edge detection
            H:  Y_D = B; // move
        endcase


    // // FSM outputsovement: load Y
    always @ (*)
    begin
        // default assignments
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; VGA_COLOR = colour; plot = 1'b0;
        Ex = 1'b0; Ey = 1'b0; Tdir_Y = 1'b0; Tdir_X = 1'b0;

        case (y_Q)
            A:  begin Lxc = 1'b1; Lyc = 1'b1; end
            B:  begin Exc = 1'b1; plot = 1'b1; end   // color a pixel
            C:  begin Lxc = 1'b1; Eyc = 1'b1; end
            D:  Lyc = 1'b1;
            E:  begin Exc = 1'b1; VGA_COLOR = ALT; plot = 1'b1; end   // color a pixel
            F:  begin Lxc = 1'b1; Eyc = 1'b1; end
            G:  begin 
                Lyc = 1'b1; 
                // Tdir_Y = (Y == 7'd0) || (Y == YSCREEN- YDIM);  // Flip Ydir at vertical edges
                // Tdir_X = (X == 8'd0) || (X == XSCREEN- XDIM);  // Flip Xdir at horizontal edges

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

            H:  
            begin
            if (drawBodyCount != 0)
                drawBodyCount <= drawBodyCount - 1;  // Move to draw the next square
            else
                begin
                drawBodyCount <= 4;

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

            // // Draw the stationary square (fixed position)
            // if (VGA_X >= Xapple && VGA_X < Xapple + XDIM && VGA_Y >= Yapple && VGA_Y < Yapple + YDIM) 
            // begin
            //     plot = 1'b1;          // enable plotting
            //     VGA_COLOR = 3'b100;   // Set the color for the stationary square
            // end

            end
        endcase
    end

    always @(posedge CLOCK_50)
        if (!SW[9])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

    assign go = ~SW[7];


    reg [7:0] VGA_X_reg, VGA_Y_reg;

    always @(*) begin
        VGA_X_reg = XSnakeLong[8 * drawBodyCount - 1 : 8 * drawBodyCount - 1 - 8] + XC;  // Dynamic part-select
        VGA_Y_reg = YSnakeLong[7 * drawBodyCount - 1 : 7 * drawBodyCount - 1 - 7] + YC;  // Dynamic part-select
    end

    assign VGA_X = VGA_X_reg;
    assign VGA_Y = VGA_Y_reg;
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

module shift_register_move_snake (clk, enable, data, data_in, data_out);
    input clk;
    input enable;
    // input maxLength; 
    input [ n * maxLength-1 :0 ] data;
    input [ n-1 :0 ] data_in;
    output reg [ n * maxLength-1 :0 ] data_out;

    parameter n = 8;
    parameter maxLength = 4;

    always @(posedge clk) 
    begin
        if (enable) begin
            // left is the head
            // add new data to the front the rest follows
            data_out <= {data_in, data[n * maxLength-1 : n-1 ]};
        end
    end

endmodule