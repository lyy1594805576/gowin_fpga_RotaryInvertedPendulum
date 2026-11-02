`timescale 1ns/1ns
module pid_fixed_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0, clear=1, start=0, hold_i=0;
reg signed [31:0] error=0, derivative=0, int_limit=10000;
reg signed [15:0] kp=1550, ki=0, kd=31600, out_limit=100;
wire signed [31:0] value;
wire done, saturated;
pid_fixed dut(.clk_50m(clk),.rst_n(rst_n),.clear(clear),.start(start),
    .hold_integrator(hold_i),.error_q8(error),.derivative_q8(derivative),
    .kp(kp),.ki(ki),.kd(kd),.int_limit(int_limit),.out_limit(out_limit),
    .value_q8(value),.done(done),.saturated(saturated));
task reset_integral;
begin
    @(negedge clk); clear=1;
    @(negedge clk); clear=0;
end
endtask
task calculate;
begin
    @(negedge clk); start=1;
    @(negedge clk); start=0;
    wait(done); #1;
end
endtask
integer positive;
initial begin
    #25; rst_n=1; reset_integral;
    if(dut.integral_q8 !== 0 || value !== 0) $fatal(1,"reset left an unknown integral/output");
    error=256; calculate; positive=value;
    error=-256; calculate;
    if(value !== -positive || positive != 97) $fatal(1,"positive/negative Q8 rounding mismatch");
    $display("PASS symmetric fractional output: +%0d / %0d Q8",positive,value);
    reset_integral; kp=0; ki=1; kd=0; int_limit=2;
    error=3*256; calculate;
    if(dut.integral_q8 !== 512) $fatal(1,"new integral sum exceeded positive limit");
    error=-10*256; calculate;
    if(dut.integral_q8 !== -512) $fatal(1,"new integral sum exceeded negative limit");
    $display("PASS integral clamps at both limits");
    reset_integral; kp=0; ki=4096; out_limit=2; int_limit=100; error=256;
    calculate; calculate; calculate;
    if(!saturated || dut.integral_q8 !== 512 || value !== 512)
        $fatal(1,"integrator accumulated farther into saturation");
    error=-256; calculate;
    if(saturated || dut.integral_q8 !== 256 || value !== 256)
        $fatal(1,"integrator could not recover from saturation");
    hold_i=1; error=256; calculate;
    if(dut.integral_q8 !== 256) $fatal(1,"integrator hold ignored");
    hold_i=0;
    $display("PASS saturation, recovery and integral hold");
    reset_integral; kp=32767; ki=0; kd=32767; out_limit=100;
    error=32'sd1000000000; derivative=32'sd1000000000; calculate;
    if(value !== 25600 || !saturated) $fatal(1,"wide positive arithmetic overflow");
    error=-32'sd1000000000; derivative=-32'sd1000000000; calculate;
    if(value !== -25600 || !saturated) $fatal(1,"wide negative arithmetic overflow");
    $display("PASS full-width signed multiplication and saturation");
    @(negedge clk); start=1;
    @(negedge clk); start=0; clear=1;
    repeat(8) @(negedge clk);
    if(done || value !== 0 || dut.integral_q8 !== 0) $fatal(1,"clear did not cancel pending PID");
    $display("PASS clear cancels pending calculation");
    $display("PASS pid_fixed_tb"); $finish;
end
initial begin #20000; $fatal(1,"PID test timeout"); end
endmodule
