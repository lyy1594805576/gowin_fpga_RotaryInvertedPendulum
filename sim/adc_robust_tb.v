`timescale 1ns/1ns
// Oracle test for the dense ADC window front end: 60 conversions per window,
// six removed from each end, control output = expanded mean of the 48 kept
// conversions, raw telemetry = expanded mean of the last 16 conversions.
module adc_robust_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0;
reg [9:0] data=739;
wire [11:0] raw, robust, bypass;
wire valid;
adc #(.SAMPLE_STRIDE(14'd8)) dut(
    .clk_50m(clk),.rst_n(rst_n),.adc_data_in(data),.adc_clk(),.adc_oe_n(),.adc_stby_n(),
    .theta_out(raw),.theta_clean(robust),.sample_valid(valid));
adc #(.SAMPLE_STRIDE(14'd8),.TRIM_ENABLE(0)) untrimmed(
    .clk_50m(clk),.rst_n(rst_n),.adc_data_in(data),.adc_clk(),.adc_oe_n(),.adc_stby_n(),
    .theta_out(),.theta_clean(bypass),.sample_valid());
localparam N=60;
integer values[0:N-1], sorted[0:N-1];
integer i,j,k,temp,raw_sum,trim_sum,windows=0;
reg [31:0] rng=32'h26101801;

task push;
input integer v;
begin
    @(negedge clk); data=v[9:0];
    while(!(dut.adc_phase==2'd3 && dut.slot_cnt==11'd1)) @(negedge clk);
    @(posedge clk);
end
endtask

task check_window;
integer window_sum;
begin
    raw_sum=0; window_sum=0;
    for(i=0;i<N;i=i+1) begin
        window_sum=window_sum+values[i];
        sorted[i]=values[i];
        if(i>=N-16) raw_sum=raw_sum+values[i];
    end
    for(i=0;i<N;i=i+1) for(j=i+1;j<N;j=j+1) begin
        if(sorted[j]<sorted[i]) begin temp=sorted[i];sorted[i]=sorted[j];sorted[j]=temp;end
    end
    trim_sum=0; for(i=6;i<N-6;i=i+1) trim_sum=trim_sum+sorted[i];
    @(negedge clk);
    if(valid!==1 || raw!==raw_sum/4 || robust!==(trim_sum+6)/12 || bypass!==raw_sum/4)
        $fatal(1,"window %0d: raw=%0d exp=%0d robust=%0d exp=%0d bypass=%0d exp=%0d valid=%b",
            windows,raw,raw_sum/4,robust,(trim_sum+6)/12,bypass,raw_sum/4,valid);
    @(posedge clk); #1;
    if(valid!==0) $fatal(1,"window %0d: valid pulse wider than one clock",windows);
    windows=windows+1;
end
endtask

task fill;
input integer value;
begin for(k=0;k<N;k=k+1) values[k]=value;end
endtask

reg [11:0] din=0;
reg fv=0;
wire [11:0] filtered, mean_only;
adc_spike_filter filter(.clk_50m(clk),.rst_n(rst_n),.din(din),.sample_valid(fv),.dout(filtered));
adc_spike_filter #(.SPIKE_THRESHOLD(0)) filter_bypass(
    .clk_50m(clk),.rst_n(rst_n),.din(din),.sample_valid(fv),.dout(mean_only));
task filter_sample;
input [11:0] x, expected;
begin
    @(negedge clk);din=x;fv=1;
    @(negedge clk);fv=0;
    if(filtered!==expected) $fatal(1,"selective median: input=%0d output=%0d expected=%0d",x,filtered,expected);
end
endtask

initial begin
    repeat(2) @(negedge clk);rst_n=1;
    fill(739);
    for(k=0;k<N;k=k+1) push(values[k]);
    check_window;
    if(robust!==2956) $fatal(1,"constant window robust=%0d expected 2956",robust);
    // Up to six impulses per side are removed; every position including the
    // last conversion of the window and mixed polarity must be covered.
    for(integer pos=0;pos<N;pos=pos+1) begin
        fill(739);values[pos]=0;
        for(k=0;k<N;k=k+1) push(values[k]);
        check_window;
        fill(739);values[pos]=1023;values[(pos+1)%N]=1023;values[(pos+2)%N]=1023;
        values[(pos+3)%N]=1023;values[(pos+4)%N]=1023;values[(pos+5)%N]=1023;
        for(k=0;k<N;k=k+1) push(values[k]);
        check_window;
        if(robust!==2956) $fatal(1,"six high conversions survived trimming at pos %0d",pos);
        fill(739);values[pos]=0;values[(pos+17)%N]=0;values[(pos+31)%N]=0;
        values[(pos+43)%N]=0;values[(pos+51)%N]=0;values[(pos+59)%N]=0;
        for(k=0;k<N;k=k+1) push(values[k]);
        check_window;
        if(robust!==2956) $fatal(1,"six low conversions survived trimming at pos %0d",pos);
        fill(739);values[pos]=0;values[(pos+30)%N]=1023;
        for(k=0;k<N;k=k+1) push(values[k]);
        check_window;
        if(robust!==2956) $fatal(1,"mixed impulses survived trimming at pos %0d",pos);
    end
    fill(0);for(k=0;k<N;k=k+1) push(values[k]);check_window;
    fill(1023);for(k=0;k<N;k=k+1) push(values[k]);check_window;
    // A seventh impulse must survive: the trim depth is exactly six per side.
    fill(739);for(k=0;k<7;k=k+1) values[k]=1023;
    for(k=0;k<N;k=k+1) push(values[k]);
    check_window;
    if(robust<=2956) $fatal(1,"seventh impulse was silently removed");
    // Linear ramp: the symmetric trim keeps the window centre unchanged.
    for(k=0;k<N;k=k+1) values[k]=600+k*5;
    for(k=0;k<N;k=k+1) push(values[k]);
    check_window;
    if(robust!==2990) $fatal(1,"ramp window robust=%0d expected 2990",robust);
    // Random windows, including the raw telemetry crossing window boundaries.
    for(integer trial=0;trial<100;trial=trial+1) begin
        for(k=0;k<N;k=k+1) begin
            rng=(rng<<1) ^ (rng[31] ? 32'h04c11db7 : 32'd0);values[k]=rng[9:0];
        end
        for(k=0;k<N;k=k+1) push(values[k]);
        check_window;
    end
    $display("PASS %0d dense ADC windows: sorted software oracle, rails, all impulse positions, 7th-impulse visibility, ramp, bypass",windows);

    filter_sample(2956,2956);
    filter_sample(2928,2956);
    if(mean_only!==2942) $fatal(1,"median bypass changed original average");
    filter_sample(2956,2956);filter_sample(2956,2956);
    filter_sample(4095,2956);filter_sample(2956,2956);filter_sample(2956,2956);
    filter_sample(0,2956);filter_sample(2956,2956);filter_sample(2956,2956);
    // At threshold, a real ramp passes with exactly the old two-tap response.
    filter_sample(2968,2962);filter_sample(2980,2974);filter_sample(2992,2986);
    // True step: only first sample is rejected, never locked to old position.
    filter_sample(3200,2992);filter_sample(3200,3096);filter_sample(3200,3200);
    filter_sample(0,3200);filter_sample(0,1600);filter_sample(0,0);
    filter_sample(4095,0);filter_sample(4095,2048);filter_sample(4095,4095);
    filter_sample(4083,4089);filter_sample(4095,4089);
    @(negedge clk);din=1234;repeat(20) @(negedge clk);
    if(filtered!==4089) $fatal(1,"filter changed without valid");
    rst_n=0;@(negedge clk);
    if(filtered!==0 || robust!==0 || raw!==0 || valid!==0) $fatal(1,"reset did not clear filter/ADC");
    rst_n=1;filter_sample(1234,1234);
    $display("PASS temporal filter: isolated spikes, no extra ramp lag, bounded step recovery, rails, reset, valid gating");
    $display("PASS adc_robust_tb");$finish;
end
initial begin #20000000;$fatal(1,"robust ADC test timeout");end
endmodule
