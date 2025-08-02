module Matrix_mult(
    input clk,
    input is_matrix_mult,             
    input [31:0] A,
    input [31:0] B,
    output reg [31:0] C,
    output reg done
);
    reg [7:0] A_mat[0:1][0:1];
    reg [7:0] B_mat[0:1][0:1];
    reg [7:0] C_mat[0:1][0:1];
    integer i, j, k;
    reg [7:0] sum;

    reg computing;

    always @(posedge clk) begin
        if (is_matrix_mult) begin
            
            A_mat[0][0] <= A[7:0];
            A_mat[0][1] <= A[15:8];
            A_mat[1][0] <= A[23:16];
            A_mat[1][1] <= A[31:24];

            B_mat[0][0] <= B[7:0];
            B_mat[0][1] <= B[15:8];
            B_mat[1][0] <= B[23:16];
            B_mat[1][1] <= B[31:24];

            computing <= 1;
            done <= 0;
        end
        else if (computing) begin
            // Compute matrix product
            for (i = 0; i < 2; i = i + 1) begin
                for (j = 0; j < 2; j = j + 1) begin
                    sum = 0;
                    for (k = 0; k < 2; k = k + 1) begin
                        sum = sum + (A_mat[i][k] * B_mat[k][j]);
                    end
                    C_mat[i][j] <= sum[7:0];
                end
            end

            // Pack result
            
            C <= {C_mat[1][1], C_mat[1][0], C_mat[0][1], C_mat[0][0]};
            done <= 1;
            computing <= 0;
        end
    end
endmodule
