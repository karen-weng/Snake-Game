`timescale 1ns / 1ns

module testbench ( );

// CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
// 				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK

	parameter CLOCK_PERIOD = 20; // clock 50

    reg [9:0] SW;
    reg [3:0] KEY;
    reg CLOCK_50;
    wire [9:0] LEDR;

	initial begin
        CLOCK_50 <= 1'b0;
	end // initial
	always @ (*)
	begin : Clock_Generator
		#((CLOCK_PERIOD) / 2) CLOCK_50 <= ~CLOCK_50;
	end
	
	initial begin
        KEY[0] <= 1'b0;
        #10 KEY[0] <= 1'b1;
	end // initial

    // In the setup below we assume that the half_sec_enable signal coming
    // from the half-second clock is asserted every 3 cycles of CLOCK_50. Of
    // course, in the real circuit the half-second clock is asserted every
    // 25M cycles. The setup below produces the Morse code for A (.-) followed
    // by the Morse code for B (-...).
	initial begin
        SW <= 3'b000; KEY[1] = 1; // not pressed;
        #20 KEY[1] <= 0; // pressed
        #10 KEY[1] <= 1; // not pressed
        #190 SW <= 3'b001;
        #10 KEY[1] <= 0; // pressed
        #10 KEY[1] <= 1; // not pressed
	end // initial
	part3 U1 (SW, CLOCK_50, KEY, LEDR);

endmodule
