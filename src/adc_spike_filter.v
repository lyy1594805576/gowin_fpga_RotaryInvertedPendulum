`timescale 1ns/1ns
// Selective median-of-three followed by the existing rounded two-point mean.
// Only a sample outside the previous two raw samples' range by more than
// SPIKE_THRESHOLD is replaced. Slow motion retains the old 1.5 ms group delay;
// a sustained large step is admitted on its second sample (no slew-rate lock).
module adc_spike_filter #(
    parameter WIDTH = 12,
    // Expanded ADC counts: 12 counts = 3 native 10-bit ADC counts.
    // Zero bypasses spike rejection, retaining just the two-point mean.
    parameter [WIDTH-1:0] SPIKE_THRESHOLD = 12'd12
)(
    input clk_50m, input rst_n,
    input [WIDTH-1:0] din, input sample_valid,
    output reg [WIDTH-1:0] dout
);
reg initialized;
reg [WIDTH-1:0] raw_prev1, raw_prev2, clean_prev;
wire [WIDTH-1:0] lower = raw_prev1 < raw_prev2 ? raw_prev1 : raw_prev2;
wire [WIDTH-1:0] upper = raw_prev1 > raw_prev2 ? raw_prev1 : raw_prev2;
wire [WIDTH-1:0] median = din < lower ? lower : (din > upper ? upper : din);
// Extend before addition, so thresholds near either rail cannot wrap around.
wire outlier = ({1'b0,din} > {1'b0,upper}+{1'b0,SPIKE_THRESHOLD}) ||
    ({1'b0,din}+{1'b0,SPIKE_THRESHOLD} < {1'b0,lower});
wire [WIDTH-1:0] clean = (SPIKE_THRESHOLD != 0 && outlier) ? median : din;
wire [WIDTH:0] sum = {1'b0,clean}+{1'b0,clean_prev}+1'b1;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        initialized <= 0; raw_prev1 <= 0; raw_prev2 <= 0;
        clean_prev <= 0; dout <= 0;
    end else if(sample_valid) begin
        raw_prev1 <= din;
        if(!initialized) begin
            raw_prev2 <= din; clean_prev <= din; dout <= din;
            initialized <= 1;
        end else begin
            // Raw history deliberately includes rejected samples: a real step
            // must be accepted next time instead of being held indefinitely.
            raw_prev2 <= raw_prev1; clean_prev <= clean;
            dout <= sum[WIDTH:1];
        end
    end
end
endmodule
