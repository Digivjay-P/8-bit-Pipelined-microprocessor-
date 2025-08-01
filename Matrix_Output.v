module Matrix_Output(
    input clk,
    input reset,
    input [31:0] C,
    input is_matrix_mult,
    //input done,
    output reg [2:0] destreg,
    output reg [7:0] wrtdata,
    output reg matrix_write_in_progress
);

reg [1:0] write_counter;


always @(posedge clk or posedge reset) begin
    if (reset) begin
        
        destreg <= 0;
        wrtdata <= 0;
        write_counter <= 0;
        matrix_write_in_progress <= 0;
    end else begin
        
        if (is_matrix_mult && !matrix_write_in_progress) begin
            matrix_write_in_progress <= 1;
            write_counter <= 0;
        end

        if (matrix_write_in_progress) begin
            case (write_counter)
                2'd0: begin destreg <= 3'd0; wrtdata <= C[7:0]; end
                2'd1: begin destreg <= 3'd1; wrtdata <= C[15:8]; end
                2'd2: begin destreg <= 3'd2; wrtdata <= C[23:16]; end
                2'd3: begin destreg <= 3'd3; wrtdata <= C[31:24]; end
            endcase

            write_counter <= write_counter + 1;

            if (write_counter == 2'd3) begin
                matrix_write_in_progress <= 0;
                
            end
        end
    end
end

endmodule

