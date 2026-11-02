`timescale 1ns/1ns
module motor_ctrl #(
    parameter [12:0] PWM_PERIOD = 13'd5400,
    // Preserve the original 20*duty scaling: 255 maps to 5100/5400 clocks.
    parameter [12:0] PWM_SPAN = 13'd5100
)(
    input clk_50m, input rst_n,
    input [1:0] motor_a_sel, input [7:0] duty_out,
    output ain1, output ain2, output pwm
);
reg [12:0] pwm_cnt;
reg [1:0] active_sel;
reg [12:0] active_cmp;
// Register each conversion stage. For n < 2^21, n/255 can be computed exactly
// by folding base-256 digits (256 = 255+1), avoiding a long divider path.
reg [7:0] duty_r;
reg [20:0] product_r, numerator_r;
reg [13:0] quotient_base_r;
reg [9:0] residual_r;
reg [12:0] requested_cmp;
reg [1:0] sel0, sel1, sel2, sel3, requested_sel;
wire [1:0] quotient_correction = residual_r >= 510 ? 2'd2 :
    residual_r >= 255 ? 2'd1 : 2'd0;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        duty_r <= 0; product_r <= 0; numerator_r <= 0;
        quotient_base_r <= 0; residual_r <= 0; requested_cmp <= 0;
        sel0 <= 0; sel1 <= 0; sel2 <= 0; sel3 <= 0; requested_sel <= 0;
    end else begin
        duty_r <= duty_out; sel0 <= motor_a_sel;
        product_r <= duty_r*PWM_SPAN; sel1 <= sel0;
        numerator_r <= product_r+21'd127; sel2 <= sel1;
        quotient_base_r <= {1'b0,numerator_r[20:8]}+{9'b0,numerator_r[20:16]};
        residual_r <= {2'b0,numerator_r[7:0]}+{2'b0,numerator_r[15:8]}+{5'b0,numerator_r[20:16]};
        sel3 <= sel2;
        requested_cmp <= quotient_base_r[12:0]+{{11{1'b0}},quotient_correction};
        requested_sel <= sel3;
    end
end
wire disabled = (motor_a_sel == 2'b00) || (duty_out == 0);
// Mid-frame guard: block stale polarity until the next frame boundary, where
// the new polarity is applied directly. A full neutral frame before reversing
// added up to one PWM period (~108us) of dead time at every zero crossing,
// which the verified pid_control reference does not have and which showed up
// as jitter around the balance point.
wire reversing = ((active_sel == 2'b01) && (motor_a_sel == 2'b10)) ||
                 ((active_sel == 2'b10) && (motor_a_sel == 2'b01));
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pwm_cnt <= 0; active_sel <= 0; active_cmp <= 0;
    end else begin
        if(pwm_cnt == PWM_PERIOD-1'b1) begin
            pwm_cnt <= 0;
            if(disabled) begin
                active_sel <= 0; active_cmp <= 0;
            end else if(requested_sel != motor_a_sel) begin
                active_sel <= 0; active_cmp <= 0;
            end else begin
                active_sel <= requested_sel;
                active_cmp <= requested_cmp;
            end
        end else pwm_cnt <= pwm_cnt+1'b1;
        if(disabled) begin active_sel <= 0; active_cmp <= 0; end
    end
end
assign ain1 = rst_n && !disabled && !reversing && active_sel[0];
assign ain2 = rst_n && !disabled && !reversing && active_sel[1];
assign pwm = rst_n && !disabled && !reversing && (pwm_cnt < active_cmp);
endmodule
