module top(CLOCK_50, SW, KEY);
    input CLOCK_50;
    input [9:0] SW;
    input [3:0] KEY;

    wire [7:0] X;
    wire [6:0] Y;
    wire [7:0] XC;
    wire [6:0] YC;

    wire MaxLength = 5;
    wire [8*MaxLength - 1:0] XSnakeLong;
    wire [7* MaxLength - 1:0] YSnakeLong;

    shift_register_move_snake S0 (CLOCK_50, SW[9], maxLength, XSnakeLong, X, XSnakeLong);
        defparam S0.n = 8; 
    shift_register_move_snake S1 (CLOCK_50, SW[9], maxLength, YSnakeLong, Y, YSnakeLong);
        defparam S0.n = 7; 

    wire Ex, Ey, Lxc, Lyc, Exc, Eyc;
    assign Ex, Ey = 1'b1;


    UpDn_count U1 (Y0, CLOCK_50, SW[9], Ey, ~SW[8], Ydir, Y); // Sw[9] reset Sw[8] load
        defparam U1.n = 7;

    UpDn_count U2 (X0, CLOCK_50, SW[9], Ex, ~SW[8], Xdir, X);
        defparam U2.n = 8;

    UpDn_count U5 ({K{1'b0}}, CLOCK_50, SW[9], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);

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

module shift_register_move_snake (clk, enable, maxLength, data, data_in, data_out);
    input clk;
    input enable;
    input maxLength; 
    input [ n * maxLength-1 :0 ] data;
    input [ n-1 :0 ] data_in;
    output reg [ n * maxLength-1 :0 ] data_out;

    parameter n = 8;

    always @(posedge clk) 
    begin
        if (enable) begin
            // left is the head
            // add new data to the front the rest follows
            data_out <= {data_in, data[n * maxLength-1 : n-1 ]};
        end
    end

endmodule

// call snake
// shift_register_move_snake S0 (CLOCK_50, enable, maxLength, data_in, data_out);
// defparam S0.n = 8; 
// or 
// defparam S0.n = 7; 