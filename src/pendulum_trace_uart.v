`timescale 1ns/1ns
// 100Hz USB telemetry v3. A6 6A 03 6F, 106-byte little-endian payload.
// v2 payload followed by command seq, applied seq, result, bank, six signed16
// stored PID gains. CRC8/ATM over the first110 bytes. 111 bytes <10ms at115200.
// HC04 keeps the original independent 9600 baud ASCII stream.
module pendulum_trace_uart #(
    parameter [18:0] PERIOD_CNT_MAX = 19'd499999,
    parameter [31:0] FIRMWARE_ID = 32'h26100801
)(
    input clk_50m, input rst_n,
    input [5:0] state, input manual_mode, input [15:0] flags,
    input [15:0] arm,
    input signed [15:0] location, pos_target,
    input [11:0] theta_raw, theta,
    input signed [31:0] angle_target_q8, pos_output_q8, angle_error_q8,
    input signed [31:0] d_angle_q8, d_pos_q8,
    input signed [15:0] u_duty, requested_pwm,
    input signed [31:0] angle_integral_q8, pos_integral_q8,
    input [223:0] config_words,
    input [15:0] pid_command_sequence, pid_active_sequence,
    input [7:0] pid_command_result, pid_command_bank,
    input [95:0] pid_readback_gains,
    output tx
);
reg [18:0] period_cnt;
reg [15:0] ms_cnt, packet_seq;
reg [31:0] uptime_ms;
reg [11:0] theta_min, theta_max;
wire [11:0] minimum = theta < theta_min ? theta : theta_min;
wire [11:0] maximum = theta > theta_max ? theta : theta_max;
reg [879:0] snapshot;
wire [879:0] live_packet = {
    pid_readback_gains,pid_command_bank,pid_command_result,pid_active_sequence,pid_command_sequence,
    config_words,
    pos_integral_q8,angle_integral_q8,requested_pwm,u_duty,d_pos_q8,d_angle_q8,
    angle_error_q8,pos_output_q8,angle_target_q8,
    4'b0,maximum,4'b0,minimum,4'b0,theta,4'b0,theta_raw,
    pos_target,location,arm,flags,{7'b0,manual_mode},{2'b0,state},
    uptime_ms,FIRMWARE_ID,packet_seq,8'd111,8'd3,8'h6a,8'ha6
};
function [7:0] crc_byte;
    input [7:0] seed, data;
    reg [7:0] c;
    integer i;
    begin
        c=seed ^ data;
        for(i=0;i<8;i=i+1) c=c[7] ? (c<<1)^8'h07 : c<<1;
        crc_byte=c;
    end
endfunction
reg sending;
reg [6:0] byte_idx;
reg [7:0] crc, byte_r;
reg flag_r;
reg gap_active;
reg [8:0] gap_cnt;
wire byte_done;
wire [6:0] next_idx = byte_idx+1'b1;
wire [7:0] next_byte = snapshot[next_idx*8 +: 8];
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        period_cnt<=0; ms_cnt<=0; uptime_ms<=0; packet_seq<=0;
        theta_min<=12'hfff; theta_max<=0;
        snapshot<=0; sending<=0; byte_idx<=0; crc<=0; byte_r<=0; flag_r<=0;
        gap_active<=0; gap_cnt<=0;
    end else begin
        flag_r<=0;
        if(ms_cnt==16'd49999) begin ms_cnt<=0; uptime_ms<=uptime_ms+1'b1; end
        else ms_cnt<=ms_cnt+1'b1;
        theta_min<=minimum; theta_max<=maximum;
        if(period_cnt==PERIOD_CNT_MAX) begin
            period_cnt<=0;
            if(!sending) begin
                snapshot<=live_packet;
                theta_min<=theta; theta_max<=theta;
                packet_seq<=packet_seq+1'b1;
                sending<=1; byte_idx<=0; byte_r<=8'ha6; flag_r<=1;
                crc<=crc_byte(8'd0,8'ha6);
            end
        end else period_cnt<=period_cnt+1'b1;
        if(sending && byte_done) begin
            if(byte_idx==7'd110) sending<=0;
            // uart_tx asserts done near the START of its stop bit. Preserve a
            // complete 434-clock stop bit before starting the following byte.
            else begin gap_active<=1; gap_cnt<=9'd433; end
        end
        if(gap_active) begin
            if(gap_cnt != 0) gap_cnt<=gap_cnt-1'b1;
            else begin
                gap_active<=0;
                byte_idx<=next_idx; flag_r<=1;
                if(byte_idx==7'd109) byte_r<=crc;
                else begin byte_r<=next_byte; crc<=crc_byte(crc,next_byte); end
            end
        end
    end
end
uart_tx #(.UART_BPS(21'd115200),.CLK_FREQ(26'd50000000)) uart_inst(
    .sys_clk(clk_50m),.sys_rst_n(rst_n),.pi_data(byte_r),.pi_flag(flag_r),
    .tx(tx),.pi_done(byte_done));
endmodule
