module Matrix_mult(
    input clk,
    input reset,
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

    always @(posedge clk or posedge reset) begin
        
        if (reset) begin
        done <= 1'b0;          // Initialized on reset
        computing <= 1'b0;
        i <= 0;
        j <= 0;
        k <= 0;
        sum <= 0;
        A_mat[0][0] <= 0;
        A_mat[0][1] <= 0;
        A_mat[1][0] <= 0;
        A_mat[1][1] <= 0;

        B_mat[0][0] <= 0;
        B_mat[0][1] <= 0;
        B_mat[1][0] <= 0;
        B_mat[1][1] <= 0;
        
        C_mat[0][0] <= 0;
        C_mat[0][1] <= 0;
        C_mat[1][0] <= 0;
        C_mat[1][1] <= 0;
        
        C = 0;

        end
  else if (is_matrix_mult && !computing) begin
            // Load input matrices when matrix multiplication starts
            A_mat[0][0] <= A[7:0];
            A_mat[0][1] <= A[15:8];
            A_mat[1][0] <= A[23:16];
            A_mat[1][1] <= A[31:24];

            B_mat[0][0] <= B[7:0];
            B_mat[0][1] <= B[15:8];
            B_mat[1][0] <= B[23:16];
            B_mat[1][1] <= B[31:24];
            
            computing <= 1;
            done <= 0;  // computation started, not done yet
            sum <= 0;
            i <= 0;
            j <= 0;
            k <= 0;
            
        end
        else if (computing) begin
            // Perform the matrix multiplication
            sum <= sum + (A_mat[i][k] * B_mat[k][j]);
            if (k==1) begin
                C_mat[i][j] <= sum + (A_mat[i][k] * B_mat[k][j]);
                sum <= 0;
                    if (j==1) begin
                        j <= 0;
                        if (i == 1) begin
                            i <= 0;
                             // Pack the result matrix into output C
                            C <= {C_mat[1][1], C_mat[1][0], C_mat[0][1], C_mat[0][0]};
                            done <= 1;      // Computation finished
                            computing <= 0; 
                        end else begin
                            i <= i + 1; 
                        end     
                    end else begin
                    j <= j + 1;
                    end
                k <= 0;
            end else begin
            k <= k + 1;

        end
    end 
end
endmodule
