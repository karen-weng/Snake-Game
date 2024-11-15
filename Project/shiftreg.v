

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