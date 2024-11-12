// FSM state table to handle multiple squares
always @ (*) begin
    case (y_Q)
        A: begin
            if (!go || !sync)
                Y_D = A;
            else
                Y_D = B;  // Move to draw the first square
        end
        B: begin
            if (XC != XDIM-1)
                Y_D = B;  // Draw current square
            else
                Y_D = C;  // Done with current square, move to next one
        end
        C: begin
            if (YC != YDIM-1)
                Y_D = B;  // Move down and continue drawing current square
            else
                Y_D = D;  // Done drawing current square, go to next state
        end
        D: begin
            if (!sync)
                Y_D = D;  // Wait for sync
            else
                Y_D = E;  // Move to erasing the square
        end
        E: begin
            if (XC != XDIM-1)
                Y_D = E;  // Erase square
            else
                Y_D = F;  // Finished erasing, move to next state
        end
        F: begin
            if (YC != YDIM-1)
                Y_D = E;  // Move down and continue erasing
            else
                Y_D = G;  // Finished erasing square, move to next state
        end
        G: begin
            Y_D = H;  // Move to state H to finalize
        end
        H: begin
            if (square_index < total_squares)
                Y_D = B;  // Move to draw the next square
            else
                Y_D = A;  // Go back to state A after finishing all squares
        end
    endcase
end

// FSM outputs to draw multiple squares
always @ (*) begin
    // Default assignments
    Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; VGA_COLOR = colour; plot = 1'b0;
    Ey = 1'b0; Tdir = 1'b0;
    
    case (y_Q)
        A: begin 
            Lxc = 1'b1; 
            Lyc = 1'b1; 
        end
        B: begin 
            Exc = 1'b1; 
            plot = 1'b1;  // Draw current square pixel
        end
        C: begin
            Lxc = 1'b1; 
            Eyc = 1'b1; 
        end
        D: begin 
            Lyc = 1'b1; 
        end
        E: begin
            Exc = 1'b1; 
            VGA_COLOR = ALT; 
            plot = 1'b1;  // Erase the square pixel
        end
        F: begin
            Lxc = 1'b1; 
            Eyc = 1'b1;
        end
        G: begin
            Lyc = 1'b1; 
            Tdir = (Y == 7'd0) || (Y == YSCREEN-1); 
        end
        H: begin
            Ey = 1'b1; 
        end
    endcase
end
