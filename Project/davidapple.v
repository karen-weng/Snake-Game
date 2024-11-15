/*
*   This code draws a horizontal line on the screen and then moves the line up and down. The
*   line "bounces" off the top and bottom of the screen and reverses directions. To run the demo
*   first press/release KEY[0] to reset the circuit. Then, press/release KEY[1] to initialize
*   the (x,y) location of the line. The line color is determined by SW[2:0]. Finally, press 
*   KEY[3] to start the animation. 
*/

module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
	
	parameter WAIT = 4'b0000, DRAW_X = 4'b0001, NEXT_Y = 4'b0010, WAIT_AGAIN = 4'b0011; 
	parameter ERASE_X = 4'b0100, NEXT_ERASE_Y = 4'b0101, CHECK_BOUNCE = 4'b0110, MOVE = 4'b0111; 
	parameter DRAW_BACK_X = 4'b1000, NEXT_BACK_Y = 4'b1001;
	parameter XSCREEN = 160, YSCREEN = 120;
	parameter XDIM = 4, YDIM = 4;
	parameter X0 = 8'd39, Y0 = 7'd5;
	parameter ALT = 3'b000; // alternate object color
	parameter K = 20; // animation speed: use 20 for hardware, 2 for ModelSim

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
	wire [7:0] X;
	wire [6:0] Y;
	wire [7:0] currentX;
	wire [6:0] currentY;
	wire [K-1:0] slowCounter;
	wire drawFrame;
	reg loadCurrentX, loadCurrentY, enableCurrentX, enableCurrentY;
	wire moveDown, moveRight;
	reg [3:0] state, next;
	
	wire [2:0] rightSpeed, downSpeed, leftSpeed, upSpeed;
	reg enableMove, enableSpeedRight, enableSpeedDown, enableSpeedLeft, enableSpeedUp;
	
	assign colour = 3'b101;
	
	wire [7:0] X1;
	wire [6:0] Y1;

	//move logic
	DoubleCounter #(.n(7)) moveBallY (.load(Y0), .limit(7'd120), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableMove), .loadEnable(~KEY[1]), .speed1(downSpeed), .speed2(upSpeed), .Q(Y1));
	DoubleCounter #(.n(8)) moveBallX (.load(X0), .limit(8'd160), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableMove), .loadEnable(~KEY[1]), .speed1(rightSpeed), .speed2(leftSpeed), .Q(X1));
	
	DFlipFlop #(.n(7)) updateY (.D(Y1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableMove), .Q(Y));
	DFlipFlop #(.n(8)) updateX (.D(X1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableMove), .Q(X));
	
	UpDownCounter #(.n(3)) changeSpeedRight (.load(3'b1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableSpeedRight), .loadEnable(~KEY[1]), .incrementUp(SW[6]), .speed(3'd1), .Q(rightSpeed));
	UpDownCounter #(.n(3)) changeSpeedDown (.load(3'b1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableSpeedDown), .loadEnable(~KEY[1]), .incrementUp(SW[5]), .speed(3'd1), .Q(downSpeed));
	UpDownCounter #(.n(3)) changeSpeedLeft (.load(3'b1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableSpeedLeft), .loadEnable(~KEY[1]), .incrementUp(SW[4]), .speed(3'd1), .Q(leftSpeed));
	UpDownCounter #(.n(3)) changeSpeedUp (.load(3'b1), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableSpeedUp), .loadEnable(~KEY[1]), .incrementUp(SW[3]), .speed(3'd1), .Q(upSpeed));
		
	//Draw Object Logic
	UpDownCounter #(.n(8)) incrementX (.load(8'd0), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableCurrentX), 
										.loadEnable(loadCurrentX), .incrementUp(1'b1), .speed(8'd1), .Q(currentX));	
	UpDownCounter #(.n(7)) incrementY (.load(7'd0), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableCurrentY),
										.loadEnable(loadCurrentY), .incrementUp(1'b1), .speed(7'd1), .Q(currentY));
	UpDownCounter #(.n(K)) whenToDraw (.load({K{1'b0}}), .clk(CLOCK_50), .resetn(KEY[0]), .enable(1'b1),
										.loadEnable(1'b0), .incrementUp(1'b1), .speed(20'd1), .Q(slowCounter));
										
	wire [7:0] backgroundX;
	wire [6:0] backgroundY;
	wire [2:0] backgroundColour;
	reg loadBackgroundX, loadBackgroundY, enableBackgroundX, enableBackgroundY;
										
	UpDownCounter #(.n(8)) incrBackgroundX (.load(8'd0), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableBackgroundX), 
										.loadEnable(loadBackgroundX), .incrementUp(1'b1), .speed(8'd1), .Q(backgroundX));	
	UpDownCounter #(.n(7)) incrBackgroundY (.load(7'd0), .clk(CLOCK_50), .resetn(KEY[0]), .enable(enableBackgroundY),
										.loadEnable(loadBackgroundY), .incrementUp(1'b1), .speed(7'd1), .Q(backgroundY));
	
	corner1 background (.address(160*backgroundY+backgroundX), .clock(CLOCK_50), .q(backgroundColour));

	assign drawFrame = (slowCounter == 0);

    // FSM state table
	always @ (*)
		case (state)
			WAIT:  if (KEY[3] || !drawFrame) next = WAIT;
				 else next = DRAW_BACK_X;
			DRAW_BACK_X : if (backgroundX != XSCREEN-1) next = DRAW_BACK_X;
				 else next = NEXT_BACK_Y;
			NEXT_BACK_Y: if (backgroundY != YSCREEN-1) next = DRAW_BACK_X;
				 else next = DRAW_X;
			DRAW_X:  if (currentX != XDIM-1) next = DRAW_X;    // draw
				 else next = NEXT_Y;
			NEXT_Y:  if (currentY != YDIM-1) next = DRAW_X;
				 else next = WAIT_AGAIN;
			WAIT_AGAIN:  if (!drawFrame) next = WAIT_AGAIN;
				 else next = ERASE_X;
			ERASE_X:  if (currentX != XDIM-1) next = ERASE_X;    // erase
				 else next = NEXT_ERASE_Y;
			NEXT_ERASE_Y:  if (currentY != YDIM-1) next = ERASE_X;
				 else next = CHECK_BOUNCE;
			CHECK_BOUNCE:  next = MOVE;
			MOVE:  next = DRAW_BACK_X;
		endcase
		
// FSM outputs
	always @ (*)
	 begin
        // default assignments
        loadCurrentX = 1'b0; loadCurrentY = 1'b0; enableCurrentX = 1'b0; enableCurrentY = 1'b0; VGA_COLOR = colour; plot = 1'b0;
        enableMove = 1'b0; enableSpeedRight = 1'b0; enableSpeedDown  = 1'b0; enableSpeedLeft = 1'b0; enableSpeedUp = 1'b0;
		  loadBackgroundX = 1'b0; loadBackgroundY = 1'b0; enableBackgroundX = 1'b0; enableBackgroundY = 1'b0;
        case (state)
            WAIT:  begin loadCurrentX = 1'b1; loadCurrentY = 1'b1; loadBackgroundX = 1'b1; loadBackgroundY = 1'b1;end
				DRAW_BACK_X: begin enableBackgroundX = 1'b1; VGA_COLOR = backgroundColour; plot = 1'b1; end
				NEXT_BACK_Y: begin loadBackgroundX = 1'b1; enableBackgroundY = 1'b1; end
            DRAW_X:  begin enableCurrentX = 1'b1; plot = 1'b1; end   // color a pixel
            NEXT_Y:  begin loadCurrentX = 1'b1; enableCurrentY = 1'b1; end
            WAIT_AGAIN:  begin loadCurrentY = 1'b1; end
            ERASE_X:  begin enableCurrentX = 1'b1; VGA_COLOR = ALT; plot = 1'b1; end   // color a pixel
            NEXT_ERASE_Y:  begin loadCurrentX = 1'b1; enableCurrentY = 1'b1; end
            CHECK_BOUNCE:  begin loadCurrentY = 1'b1; loadBackgroundY = 1'b1; end
            MOVE: begin
							enableMove = 1'b1;
							enableSpeedRight = (rightSpeed < 3'b111 & SW[6]) | (rightSpeed > 0 & ~SW[6]);
							enableSpeedDown = (downSpeed < 3'b111 & SW[5]) | (downSpeed > 0 & ~SW[5]);
							enableSpeedLeft = (leftSpeed < 3'b111 & SW[4]) | (leftSpeed > 0 & ~SW[4]);
							enableSpeedUp = (upSpeed < 3'b111 & SW[3]) | (upSpeed > 0 & ~SW[3]);

						end
        endcase
    end

    always @(posedge CLOCK_50)
        if (!KEY[0])
            state <= 1'b0;
        else
            state <= next;
				
    assign VGA_X = (state == DRAW_BACK_X) ? backgroundX : X + currentX;
    assign VGA_Y = (state == DRAW_BACK_X) ? backgroundY : Y + currentY;
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

module DFlipFlop #(parameter n = 8)
(
	input [n-1:0] D,
	input clk, resetn, enable,
	output reg [n-1:0] Q
);
	
	always @(posedge clk)
	begin
		if (!resetn)
			Q <= 0;
		else if (enable)
			Q <= D;
	end
endmodule

module UpDownCounter #(parameter n = 8)
(
	input [n-1:0] load,
	input clk, resetn, enable, loadEnable, incrementUp,
	input [2:0] speed,
	output reg [n-1:0] Q
);
	always @(posedge clk)
	begin
		if (!resetn)
			Q <= 0;
		else if (loadEnable)
			Q <= load;
		else if (enable)
		begin
			if (incrementUp)
				Q <= Q + speed;
			else
				Q <= Q - speed;
		end
	end
endmodule

module DoubleCounter #(parameter n = 8)
(
	input [n-1:0] load, limit,
	input clk, resetn, enable, loadEnable,
	input [2:0] speed1, speed2,
	output reg [n-1:0] Q
);
	always @(posedge clk)
	begin
		if (!resetn)
			Q <= 0;
		else if (loadEnable)
			Q <= load;
		else if (Q > limit & speed1 > speed2 & enable)
			Q <= 0;
		else if (Q > limit & speed1 < speed2 & enable)
			Q <= limit;
		else if (enable)
			Q <= Q + speed1 - speed2;
	end
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
