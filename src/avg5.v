`timescale 1ns/1ns
// Two-, three- or five-sample moving average. External sample_valid orders the
// ADC, filter and controller without an extra sample of scheduling delay.
// TAPS=2 halves the filter group delay (1.5ms instead of 3ms at 3ms/sample) to
// recover control-loop phase margin; the ADC 16-sample window average remains.
module avg5 #(
    parameter WIDTH = 12, parameter TAPS = 5, parameter EXTERNAL_SAMPLE = 0,
    parameter [17:0] SAMPLE_CNT_MAX = 18'd149999
)(
    input clk_50m, input rst_n, input [WIDTH-1:0] din,
    input sample_valid, output reg [WIDTH-1:0] dout
);
reg [17:0] cnt;
reg initialized;
reg [WIDTH-1:0] s0, s1, s2, s3;
wire sample_en = EXTERNAL_SAMPLE ? sample_valid : (cnt == SAMPLE_CNT_MAX);
wire [WIDTH+2:0] sum2 = {3'b0,din}+{3'b0,s0};
wire [WIDTH+2:0] sum3 = sum2+{3'b0,s1};
wire [WIDTH+2:0] sum5 = sum3+{3'b0,s2}+{3'b0,s3};
wire [WIDTH+2:0] average = (TAPS == 2) ? ((sum2+1) >> 1) :
    (TAPS == 3) ? ((sum3+1)/3) : ((sum5+2)/5);
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) cnt <= 0;
    else if(cnt == SAMPLE_CNT_MAX) cnt <= 0;
    else cnt <= cnt+1'b1;
end
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        s0 <= 0; s1 <= 0; s2 <= 0; s3 <= 0; dout <= 0; initialized <= 0;
    end else if(sample_en) begin
        if(!initialized) begin
            s0 <= din; s1 <= din; s2 <= din; s3 <= din;
            dout <= din; initialized <= 1;
        end else begin
            s0 <= din; s1 <= s0; s2 <= s1; s3 <= s2;
            dout <= average[WIDTH-1:0];
        end
    end
end
endmodule
