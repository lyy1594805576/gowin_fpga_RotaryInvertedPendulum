`timescale 1ns/1ns
// Robust 3PA1030 angle acquisition front end.
//
// The ADC is clocked at 12.5 MHz and produces a conversion every 80 ns; only a
// sparse set of conversions is accepted so that every accumulation window ends
// exactly on the 3 ms controller tick (150000 clocks at 50 MHz):
//
//   SAMPLE_STRIDE   clocks between accepted conversions (2500 = 50 us)
//   WINDOW_SAMPLES  accepted conversions per window (60)
//   TRIM_PER_SIDE   conversions removed from each end of the sorted window (6)
//
// SAMPLE_STRIDE * WINDOW_SAMPLES = 150000 keeps sample_valid in the same 3 ms
// phase as before.  The trimmed window mean rejects up to 6 corrupted
// conversions of either polarity while averaging 48 conversions instead of the
// previous 12, so converter/analogue white noise drops by about 2x without any
// extra control delay (the window stays 3 ms long).
//
// theta_out is the raw telemetry point: plain mean of the last 16 accepted
// conversions, expanded to 12 bits (the historical "16-sample averaged ADC"
// value).  theta_clean is the trimmed window mean, expanded to 12 bits:
//   4 * (sum of the kept conversions / 48) = (sum of kept + 6) / 12.
// TRIM_ENABLE=0 selects the plain 16-conversion mean for theta_clean as a
// historical bypass.
module adc #(
    // Clocks between accepted conversions, must be a multiple of 4 so that
    // every conversion is read at the fixed phase (+40 ns after the ADC clock
    // rising edge) and SAMPLE_STRIDE*WINDOW_SAMPLES must equal 150000.
    parameter [13:0] SAMPLE_STRIDE = 14'd2500,
    parameter TRIM_ENABLE = 1
)(
    input clk_50m, input rst_n,
    output adc_clk, output adc_oe_n, output adc_stby_n,
    input [9:0] adc_data_in,
    // Keep the historical raw mean for telemetry; control uses theta_clean.
    output reg [11:0] theta_out, output reg sample_valid,
    output reg [11:0] theta_clean
);
localparam [7:0] WINDOW_SAMPLES = 8'd60;
localparam [3:0] TRIM_PER_SIDE = 4'd6;
localparam [10:0] SLOT_MAX = SAMPLE_STRIDE[13:2];  // SAMPLE_STRIDE/4 phase slots

reg [1:0] adc_phase;
reg [10:0] slot_cnt;
reg [7:0] sample_index;
reg [15:0] sum_all;
reg [15:0] ext_sum;
reg [9:0] adc_sample;
// Six smallest and six largest accepted conversions of the current window.
// Keeping TRIM_PER_SIDE outliers per side covers about 300 us of a corrupted
// burst at the 50 us acceptance interval while the symmetric trim keeps the
// mean unbiased for slow motion and ramps.
reg [9:0] min1, min2, min3, min4, min5, min6;
reg [9:0] max1, max2, max3, max4, max5, max6;
// Sliding plain sum of the last 16 accepted conversions (raw telemetry).
reg [9:0] hist0, hist1, hist2, hist3, hist4, hist5, hist6, hist7,
          hist8, hist9, hist10, hist11, hist12, hist13, hist14, hist15;
reg [3:0] hist_idx;
reg [14:0] ring_sum;

wire [9:0] s = adc_data_in;
// Sorted insertion of the new conversion into the six-entry min/max lists.
wire [9:0] n_min1 = (s < min1) ? s : min1;
wire [9:0] n_min2 = (s < min1) ? min1 : ((s < min2) ? s : min2);
wire [9:0] n_min3 = (s < min1) ? min2 :
    ((s < min2) ? min2 : ((s < min3) ? s : min3));
wire [9:0] n_min4 = (s < min1) ? min3 : ((s < min2) ? min3 :
    ((s < min3) ? min3 : ((s < min4) ? s : min4)));
wire [9:0] n_min5 = (s < min1) ? min4 : ((s < min2) ? min4 :
    ((s < min3) ? min4 : ((s < min4) ? min4 : ((s < min5) ? s : min5))));
wire [9:0] n_min6 = (s < min1) ? min5 : ((s < min2) ? min5 :
    ((s < min3) ? min5 : ((s < min4) ? min5 :
    ((s < min5) ? min5 : ((s < min6) ? s : min6)))));
wire [9:0] n_max1 = (s > max1) ? s : max1;
wire [9:0] n_max2 = (s > max1) ? max1 : ((s > max2) ? s : max2);
wire [9:0] n_max3 = (s > max1) ? max2 :
    ((s > max2) ? max2 : ((s > max3) ? s : max3));
wire [9:0] n_max4 = (s > max1) ? max3 : ((s > max2) ? max3 :
    ((s > max3) ? max3 : ((s > max4) ? s : max4)));
wire [9:0] n_max5 = (s > max1) ? max4 : ((s > max2) ? max4 :
    ((s > max3) ? max4 : ((s > max4) ? max4 : ((s > max5) ? s : max5))));
wire [9:0] n_max6 = (s > max1) ? max5 : ((s > max2) ? max5 :
    ((s > max3) ? max5 : ((s > max4) ? max5 :
    ((s > max5) ? max5 : ((s > max6) ? s : max6)))));

// The sum of the twelve trimmed extremes is maintained incrementally so that
// the window-end result path does not include the six-level sorted-insertion
// chain: that chain only has to settle inside the 2500-clock sample interval.
// A new conversion enters the six smallest iff it is below min6, and the entry
// always drops the old min6; the six largest behave symmetrically.
wire [15:0] local_sum = sum_all + {6'b0, s};
wire [15:0] min_delta = (s < min6) ? ({6'b0, min6} - {6'b0, s}) : 16'd0;
wire [15:0] max_delta = (s > max6) ? ({6'b0, s} - {6'b0, max6}) : 16'd0;
wire [15:0] ext_next = ext_sum - min_delta + max_delta;
wire [15:0] trimmed_sum = local_sum - ext_next;
// 4 * mean(48 kept) = sum/12, rounded to nearest.
wire [15:0] clean_full = (trimmed_sum + 16'd6) / 16'd12;

wire [9:0] hist_old = (hist_idx == 4'd0) ? hist0 :
    (hist_idx == 4'd1) ? hist1 : (hist_idx == 4'd2) ? hist2 :
    (hist_idx == 4'd3) ? hist3 : (hist_idx == 4'd4) ? hist4 :
    (hist_idx == 4'd5) ? hist5 : (hist_idx == 4'd6) ? hist6 :
    (hist_idx == 4'd7) ? hist7 : (hist_idx == 4'd8) ? hist8 :
    (hist_idx == 4'd9) ? hist9 : (hist_idx == 4'd10) ? hist10 :
    (hist_idx == 4'd11) ? hist11 : (hist_idx == 4'd12) ? hist12 :
    (hist_idx == 4'd13) ? hist13 : (hist_idx == 4'd14) ? hist14 : hist15;
wire [14:0] ring_next = ring_sum + {5'b0, s} - {5'b0, hist_old};
wire [11:0] raw_next = ring_next[13:2];

assign adc_clk = adc_phase[1];
assign adc_oe_n = 1'b0;
assign adc_stby_n = 1'b0;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        adc_phase <= 2'd0; slot_cnt <= 11'd0; sample_index <= 8'd0;
        sum_all <= 16'd0; ext_sum <= 16'd6138; adc_sample <= 10'd0;
        min1 <= 10'd1023; min2 <= 10'd1023; min3 <= 10'd1023;
        min4 <= 10'd1023; min5 <= 10'd1023; min6 <= 10'd1023;
        max1 <= 10'd0; max2 <= 10'd0; max3 <= 10'd0;
        max4 <= 10'd0; max5 <= 10'd0; max6 <= 10'd0;
        hist0 <= 10'd0; hist1 <= 10'd0; hist2 <= 10'd0; hist3 <= 10'd0;
        hist4 <= 10'd0; hist5 <= 10'd0; hist6 <= 10'd0; hist7 <= 10'd0;
        hist8 <= 10'd0; hist9 <= 10'd0; hist10 <= 10'd0; hist11 <= 10'd0;
        hist12 <= 10'd0; hist13 <= 10'd0; hist14 <= 10'd0; hist15 <= 10'd0;
        hist_idx <= 4'd0; ring_sum <= 15'd0;
        theta_out <= 12'd0; theta_clean <= 12'd0; sample_valid <= 1'b0;
    end else begin
        adc_phase <= adc_phase + 1'b1;
        sample_valid <= 1'b0;
        // +40 ns after the ADC clock rising edge: the 3PA1030 output is valid
        // from about +25 ns and the next conversion appears one 80 ns later.
        if(adc_phase == 2'd3) begin
            if(slot_cnt == SLOT_MAX - 1'b1) begin
                slot_cnt <= 11'd0;
                adc_sample <= s;
                case(hist_idx)
                    4'd0: hist0 <= s; 4'd1: hist1 <= s;
                    4'd2: hist2 <= s; 4'd3: hist3 <= s;
                    4'd4: hist4 <= s; 4'd5: hist5 <= s;
                    4'd6: hist6 <= s; 4'd7: hist7 <= s;
                    4'd8: hist8 <= s; 4'd9: hist9 <= s;
                    4'd10: hist10 <= s; 4'd11: hist11 <= s;
                    4'd12: hist12 <= s; 4'd13: hist13 <= s;
                    4'd14: hist14 <= s; default: hist15 <= s;
                endcase
                hist_idx <= hist_idx + 1'b1;
                ring_sum <= ring_next;
                ext_sum <= ext_next;
                sum_all <= local_sum;
                if(sample_index == WINDOW_SAMPLES - 1'b1) begin
                    // Include conversion 60 before clearing the window.
                    theta_out <= raw_next;
                    theta_clean <= TRIM_ENABLE ? clean_full[11:0] : raw_next;
                    sample_valid <= 1'b1;
                    sample_index <= 8'd0;
                    sum_all <= 16'd0;
                    ext_sum <= 16'd6138;
                    min1 <= 10'd1023; min2 <= 10'd1023; min3 <= 10'd1023;
                    min4 <= 10'd1023; min5 <= 10'd1023; min6 <= 10'd1023;
                    max1 <= 10'd0; max2 <= 10'd0; max3 <= 10'd0;
                    max4 <= 10'd0; max5 <= 10'd0; max6 <= 10'd0;
                end else begin
                    sample_index <= sample_index + 1'b1;
                    min1 <= n_min1; min2 <= n_min2; min3 <= n_min3;
                    min4 <= n_min4; min5 <= n_min5; min6 <= n_min6;
                    max1 <= n_max1; max2 <= n_max2; max3 <= n_max3;
                    max4 <= n_max4; max5 <= n_max5; max6 <= n_max6;
                end
            end else slot_cnt <= slot_cnt + 1'b1;
        end
    end
end
endmodule
