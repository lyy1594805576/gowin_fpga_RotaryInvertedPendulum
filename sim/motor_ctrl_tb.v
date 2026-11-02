`timescale 1ns/1ns
module motor_ctrl_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0;
reg [1:0] sel=0;
reg [7:0] duty=0;
wire in1,in2,pwm;
motor_ctrl #(.PWM_PERIOD(13'd16),.PWM_SPAN(13'd16)) dut(.clk_50m(clk),.rst_n(rst_n),
    .motor_a_sel(sel),.duty_out(duty),.ain1(in1),.ain2(in2),.pwm(pwm));
task boundary;
begin
    @(negedge clk); while(dut.pwm_cnt != 15) @(negedge clk);
    @(posedge clk); #1;
end
endtask
integer i,high_count;
initial begin
    #25; rst_n=1;
    @(negedge clk); sel=1; duty=255;
    boundary;
    high_count=0;
    for(i=0;i<16;i=i+1) begin
        if(pwm) high_count=high_count+1;
        @(posedge clk); #1;
    end
    if(high_count!=16 || !in1 || in2) $fatal(1,"full-duty mapping/direction incorrect");
    $display("PASS full duty: 16/16 high clocks");
    repeat(3) @(negedge clk); duty=128; #1;
    if(dut.active_cmp !== 16) $fatal(1,"duty changed inside PWM frame");
    boundary;
    high_count=0;
    for(i=0;i<16;i=i+1) begin
        if(pwm) high_count=high_count+1;
        @(posedge clk); #1;
    end
    if(high_count!=8) $fatal(1,"half-duty mapping incorrect");
    $display("PASS boundary-only duty update and 8/16 half duty");
    repeat(3) @(negedge clk); sel=2; #1;
    if(pwm || in1 || in2) $fatal(1,"old polarity persisted on reversal");
    // Settle the request pipeline early in the frame so the boundary applies
    // the new polarity directly; a full neutral frame is no longer inserted.
    @(negedge clk); while(dut.pwm_cnt > 8) @(negedge clk);
    boundary;
    if(!in2 || in1 || !pwm) $fatal(1,"opposite polarity not applied at frame boundary");
    high_count=0;
    for(i=0;i<16;i=i+1) begin
        if(pwm) high_count=high_count+1;
        @(posedge clk); #1;
    end
    if(high_count!=8) $fatal(1,"reversed duty incorrect at frame boundary");
    $display("PASS reversal applies new polarity at frame boundary without a neutral frame");
    repeat(3) @(negedge clk); sel=0; #1;
    if(pwm || in1 || in2) $fatal(1,"stop was not immediate");
    @(negedge clk); sel=1;
    @(negedge clk); while(dut.pwm_cnt > 8) @(negedge clk);
    boundary;
    if(!in1 || in2 || !pwm) $fatal(1,"drive did not resume at frame boundary after short stop");
    duty=0; #1;
    if(pwm || in1 || in2) $fatal(1,"zero duty did not disable all outputs");
    $display("PASS immediate stop, zero duty and boundary-only reversal");
    $display("PASS motor_ctrl_tb"); $finish;
end
initial begin #20000; $fatal(1,"motor test timeout"); end
endmodule
