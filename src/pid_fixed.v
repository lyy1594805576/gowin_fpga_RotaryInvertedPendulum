`timescale 1ns/1ns
// Q12 gains; Q8 error, derivative and integral. Arithmetic is pipelined at
// 50 MHz, with symmetric rounding and conditional integration at saturation.
module pid_fixed(
    input clk_50m, input rst_n, input clear, input start,
    input hold_integrator,
    input signed [31:0] error_q8, input signed [31:0] derivative_q8,
    input signed [15:0] kp, input signed [15:0] ki, input signed [15:0] kd,
    input signed [31:0] int_limit, input signed [15:0] out_limit,
    output reg signed [31:0] value_q8,
    output reg done, output reg saturated,
    output signed [31:0] integral_value_q8
);
localparam IDLE=3'd0, INT_SUM=3'd1, INT_CLAMP=3'd2, MUL=3'd3,
           ADD_PI=3'd4, ADD_D=3'd5, RESULT=3'd6;
reg [2:0] phase;
reg signed [31:0] integral_q8, candidate_q8;
assign integral_value_q8 = integral_q8;
reg signed [31:0] error_r, derivative_r;
reg signed [15:0] kp_r, ki_r, kd_r;
reg signed [47:0] p_term, i_term, d_term;
reg signed [48:0] pi_sum;
reg signed [49:0] total, limit_q20;
reg signed [32:0] sum_r, int_limit_r;
reg hold_r;
wire signed [32:0] integral_sum =
    {integral_q8[31], integral_q8} + {error_r[31], error_r};
wire signed [32:0] limit_q8 = $signed({2'b0, int_limit[22:0], 8'b0});
wire integrating_outward = (error_r != 0) && (ki_r != 0) &&
    ((error_r[31] ^ ki_r[15]) == total[49]);
function signed [31:0] round_q20_to_q8;
    input signed [49:0] x;
    reg [49:0] magnitude, rounded;
    begin
        magnitude = x[49] ? -x : x;
        rounded = (magnitude + 50'd2048) >> 12;
        round_q20_to_q8 = x[49] ? -$signed(rounded[31:0]) : $signed(rounded[31:0]);
    end
endfunction
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        phase <= IDLE; integral_q8 <= 0; candidate_q8 <= 0;
        error_r <= 0; derivative_r <= 0; kp_r <= 0; ki_r <= 0; kd_r <= 0;
        p_term <= 0; i_term <= 0; d_term <= 0; pi_sum <= 0; total <= 0;
        limit_q20 <= 0; value_q8 <= 0; done <= 0; saturated <= 0;
        sum_r <= 0; int_limit_r <= 0; hold_r <= 0;
    end else if(clear) begin
        phase <= IDLE; integral_q8 <= 0; candidate_q8 <= 0;
        value_q8 <= 0; done <= 0; saturated <= 0;
    end else begin
        done <= 1'b0;
        case(phase)
            IDLE: if(start) begin
                error_r <= error_q8; derivative_r <= derivative_q8;
                kp_r <= kp; ki_r <= ki; kd_r <= kd;
                limit_q20 <= $signed({14'b0, out_limit[15:0], 20'b0});
                int_limit_r <= (int_limit > 0) ? limit_q8 : 33'sd0;
                hold_r <= hold_integrator;
                phase <= INT_SUM;
            end
            INT_SUM: begin
                sum_r <= integral_sum;
                phase <= INT_CLAMP;
            end
            INT_CLAMP: begin
                if(ki_r == 0 || int_limit_r == 0) candidate_q8 <= 0;
                else if(hold_r) candidate_q8 <= integral_q8;
                else if(sum_r > int_limit_r) candidate_q8 <= int_limit_r[31:0];
                else if(sum_r < -int_limit_r) candidate_q8 <= -int_limit_r[31:0];
                else candidate_q8 <= sum_r[31:0];
                phase <= MUL;
            end
            MUL: begin
                p_term <= kp_r * error_r;
                i_term <= ki_r * candidate_q8;
                d_term <= kd_r * derivative_r;
                phase <= ADD_PI;
            end
            ADD_PI: begin
                pi_sum <= {p_term[47], p_term} + {i_term[47], i_term};
                phase <= ADD_D;
            end
            ADD_D: begin
                total <= {pi_sum[48], pi_sum} + {{2{d_term[47]}}, d_term};
                phase <= RESULT;
            end
            RESULT: begin
                saturated <= (total > limit_q20) || (total < -limit_q20);
                if(total > limit_q20) value_q8 <= round_q20_to_q8(limit_q20);
                else if(total < -limit_q20) value_q8 <= -round_q20_to_q8(limit_q20);
                else value_q8 <= round_q20_to_q8(total);
                // Allow integral recovery; block only a change pushing farther
                // into saturation. Clamp the new sum, rather than the old sum.
                if(!(((total > limit_q20) || (total < -limit_q20)) && integrating_outward))
                    integral_q8 <= candidate_q8;
                done <= 1'b1; phase <= IDLE;
            end
            default: phase <= IDLE;
        endcase
    end
end
endmodule
