`timescale 1ns/1ns
// Integrates the dense ADC window (60 conversions, sparse accept) with the
// downstream moving-average filter used by the historic comparison benches.
module sampling_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0;
reg [9:0] adc_data=700;
wire [11:0] theta;
wire sample_valid;
localparam N=60;
adc #(.SAMPLE_STRIDE(14'd8)) dut(.clk_50m(clk),.rst_n(rst_n),
    .adc_data_in(adc_data),.adc_clk(),.adc_oe_n(),.adc_stby_n(),
    .theta_out(theta),.sample_valid(sample_valid),.theta_clean());
reg [11:0] din=3000;
reg filter_valid=0;
wire [11:0] filtered;
avg5 #(.TAPS(3),.EXTERNAL_SAMPLE(1)) filter(.clk_50m(clk),.rst_n(rst_n),
    .din(din),.sample_valid(filter_valid),.dout(filtered));
task filter_sample;
input [11:0] x;
begin
    @(negedge clk); din=x; filter_valid=1;
    @(negedge clk); filter_valid=0;
end
endtask
integer i;
initial begin
    #25; rst_n=1;
    // Dense window: 60 conversions, the last one differs by 16 ADC codes.
    // Raw telemetry = expanded mean of the last 16 conversions:
    // (15*700 + 716) * 4 / 16 = 2804.
    for(i=0;i<N;i=i+1) begin
        adc_data=(i==N-1) ? 716 : 700;
        repeat(8) begin @(posedge clk); #1; end
    end
    if(theta !== 2804 || sample_valid !== 1) $fatal(1,"ADC did not include exactly the last 16 conversions");
    @(posedge clk); #1;
    if(sample_valid !== 0) $fatal(1,"ADC valid pulse wider than one clock");
    $display("PASS dense ADC raw average includes the final conversion, valid pulse=1 clock");
    filter_sample(3000);
    if(filtered !== 3000) $fatal(1,"filter startup was diluted by zeros");
    filter_sample(3030);
    if(filtered !== 3010) $fatal(1,"first three-point average incorrect");
    filter_sample(3030);
    if(filtered !== 3020) $fatal(1,"second three-point average incorrect");
    filter_sample(3030);
    if(filtered !== 3030) $fatal(1,"filter did not settle after three samples");
    din=0; repeat(10) @(posedge clk); #1;
    if(filtered !== 3030) $fatal(1,"filter changed without sample_valid");
    $display("PASS filter priming, step response and valid gating");
    $display("PASS sampling_tb"); $finish;
end
initial begin #200000; $fatal(1,"sampling test timeout"); end
endmodule
