`timescale 1ns/1ns
// An ADC model publishes a new bus value 25 ns after its rising clock edge.
// Verify the FPGA reads an accepted conversion at the fixed phase, 40 ns after
// the rise, and only at the sparse SAMPLE_STRIDE cadence of the dense window
// front end (stride shortened here so the bench stays small).
module adc_phase_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0;
reg [9:0] delayed_data=0;
wire adc_clk;
wire [11:0] theta_out, theta_clean;
wire sample_valid;
adc #(.SAMPLE_STRIDE(14'd8)) dut(.clk_50m(clk),.rst_n(rst_n),
    .adc_data_in(delayed_data),.adc_clk(adc_clk),.adc_oe_n(),.adc_stby_n(),
    .theta_out(theta_out),.sample_valid(sample_valid),.theta_clean(theta_clean));
always @(posedge adc_clk) if(rst_n) begin #25; delayed_data=delayed_data+1'b1; end
time last_rise=0, previous_rise=0;
integer captures=0;
always @(posedge adc_clk) if(rst_n) begin
    if(previous_rise && $time-previous_rise != 80) $fatal(1,"ADC clock period is not 80 ns");
    last_rise=$time; previous_rise=$time;
end
// SAMPLE_STRIDE=8 -> SLOT_MAX=2: the accepted conversion is the phase-3 edge
// with slot_cnt==1, i.e. 160 ns after the previous accepted conversion.
always @(posedge clk) if(rst_n && dut.adc_phase==2'd3 && dut.slot_cnt==11'd1) begin
    if($time-last_rise != 40) $fatal(1,"ADC capture is not 40 ns after rising edge");
    #1;
    if(dut.adc_sample !== delayed_data) $fatal(1,"ADC captured old/changing bus data");
    captures=captures+1;
end
// The captured register may only change on accepted conversions.
reg [9:0] previous_sample=0;
integer changes=0;
always @(posedge clk) if(rst_n && $time>30) begin
    #2;
    if(dut.adc_sample !== previous_sample) begin
        changes=changes+1; previous_sample=dut.adc_sample;
    end
end
initial begin
    #25; rst_n=1;
    repeat(64*8) @(negedge clk);  // 512 clocks = 64 accepted conversions
    if(captures!=64) $fatal(1,"accepted captures=%0d expected 64",captures);
    if(changes!=64) $fatal(1,"adc_sample updated %0d times expected 64",changes);
    $display("PASS ADC clock=12.5 MHz; 64 sparse captures at +40 ns with +25 ns bus delay");
    $display("PASS adc_phase_tb"); $finish;
end
initial begin #200000; $fatal(1,"ADC phase test timeout"); end
endmodule
