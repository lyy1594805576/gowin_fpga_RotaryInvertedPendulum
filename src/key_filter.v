`timescale 1ns/1ns
module key_filter #(

	parameter CNT_MAX = 20'd1000_000
	
)(

	input		sys_clk		,	// 系统时钟50MHz, 来自 top 的 clk_50m
	input		sys_rst_n	,	// 复位(低有效), 来自 top 的 rst_n
	input		key_in		,	// 单个按键原始电平(按下=低), 来自 top 的 key_sw[i]

	output	reg	key_flag = 1'b0	// 按键消抖后的单周期脉冲(按下稳定20ms后拉高1拍), 送给 top 的 swing_key[i]

);

reg [19:0] cnt_20ms ; 

initial begin
	cnt_20ms <= 'd0;
end

always@(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		cnt_20ms <= 20'b0;
	else if(key_in == 1'b1)
		cnt_20ms <= 20'b0;
	else if(cnt_20ms == CNT_MAX && key_in == 1'b0)
		cnt_20ms <= cnt_20ms;
	else
		cnt_20ms <= cnt_20ms + 1'b1;
end

always@(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		key_flag <= 1'b0;
	else if(cnt_20ms == CNT_MAX - 1'b1)
		key_flag <= 1'b1;
	else
		key_flag <= 1'b0;
end




endmodule