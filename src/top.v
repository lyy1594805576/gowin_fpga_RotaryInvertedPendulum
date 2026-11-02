`timescale 1ns/1ns
module top #(
    parameter ADC_TRIM_ENABLE = 1,
    parameter [11:0] ADC_SPIKE_THRESHOLD = 12'd12,
    // Restore continuous small-angle damping after the hysteresis trial;
    // retain ADC spike rejection independently of derivative tuning.
    parameter [11:0] ANGLE_D_HYST_COUNTS = 12'd0,
    // D-only noise suppression; use 1 to restore the 0x26101704 D filter.
    parameter ANGLE_D_FILTER_SHIFT = 2,
    parameter ANGLE_D_TWO_POLE = 1,
    parameter FRICTION_FF_SOFT_ENABLE = 1,
    parameter [8:0] FRICTION_FF_POS_SCALE_Q8 = 9'd256,
    parameter [8:0] FRICTION_FF_NEG_SCALE_Q8 = 9'd256,
    parameter [3:0] FRICTION_FF_DEFAULT = 4'd0
)(

    // ---- 系统时钟与复位 (来自板载) ----
    input			clk_50m 	,	// 50MHz时钟, 来自板载晶振, 分发给所有子模块的 clk_50m/sys_clk
    input			rst_n   	,	// 复位(低有效), 来自复位按键, 分发给所有子模块的 rst_n/sys_rst_n

    // ---- 按键 (来自板载按键) ----
	input	[3:0]	key_sw		,
	input			dip_sw1		,	// 拨码1: OFF=顺时针 ON=逆时针
	input			dip_sw2		,	// 拨码2: 预留	// SW1~SW4 按键原始电平(按下=低), 送给 key_filter 消抖

    // ---- 串口 (接 USB 转串口) ----
	output			uart_tx		,	// 串口发送, 来自 uart 模块的 fpga_tx
	input			uart_rx		,	// 串口接收, 送给 uart 模块的 fpga_rx

    // ---- 蓝牙 HC-04 串口 (接排针空闲脚: hc04_rx=E20 输入, hc04_tx=D20 输出) ----
	input			hc04_rx		,	// 蓝牙接收(HC-04 TXD -> FPGA), 送 pid_cmd 解析 PID 命令
	output			hc04_tx		,	// 蓝牙发送(FPGA -> HC-04 RXD), 调试数据镜像(与 uart_tx 同帧)

    // ---- ADC WDD35D4 接口 (接角度传感器并行ADC) ----
    output			adc_clk		,	// ADC采样时钟(12.5MHz), 来自 adc 模块, 接3PA1030的CLK脚
    output			adc_oe_n	,	// ADC输出使能(低有效), 来自 adc 模块(恒0=一直使能)
    output			adc_stby_n	,	// ADC休眠(低有效), 来自 adc 模块(恒0=不休眠)
    input	[9:0]	adc_data_in	,	// ADC并行数据 D9~D0, 来自ADC芯片, 送给 adc 模块

    // ---- 编码器1 (接XH2转臂编码器，作为位置反馈及串口观测) ----
	input			enc1_a		,	// 编码器A相方波, 送给 encoder_1 的 enc_a
    input			enc1_b		,	// 编码器B相方波, 送给 encoder_1 的 enc_b

    // ---- 编码器2 (接XH1备用编码器，不参与转臂控制) ----
	input			enc2_a		,	// 编码器A相方波, 送给 encoder_2 的 enc_a
    input			enc2_b		,	// 编码器B相方波, 送给 encoder_2 的 enc_b

    // ---- 电机A (TB6612的A通道，驱动转臂) ----
	output			AN1			,	// 电机A的IN1, 来自 motor_ctrl_A 的 ain1
    output			AN2			,	// 电机A的IN2, 来自 motor_ctrl_A 的 ain2
    output			PWMA		,	// 电机A的PWM, 来自 motor_ctrl_A 的 pwm

    // ---- 电机B (TB6612的B通道，固定停止) ----
    output			BN1			,	// 电机B的IN1, 来自 motor_ctrl_B
    output			BN2			,   // 电机B的IN2, 来自 motor_ctrl_B(转臂控制)
    output			PWMB		,	// 电机B的PWM, 来自 motor_ctrl_B(转臂控制)

    // ---- LED (板载LED, 低电平点亮) ----
    output	[3:0]	led				// 4路LED, 来自 led_ctrl, 按SW1~SW4点亮对应LED

    );

//===========================================================================
// 按键消抖: 每个按键 key_sw[i] 消抖后产生单周期脉冲 swing_key[i]
//   swing_key[0]=SW1  swing_key[1]=SW2  swing_key[2]=SW3  swing_key[3]=SW4
//   用途: SW4→起摆/停止, SW1=手动 / SW2=自动 / SW3=顺时针90度; 调试串口每0.5秒自动发送
//===========================================================================
	wire			[3:0]	swing_key		;
	wire			[1:0]	ctrl_state		;	// 控制状态(0空闲/1起摆/2平衡), 由 pendulum_ctrl 输出, 送给 led_ctrl

	genvar i;
		generate for(i = 0; i <= 3; i = i + 1) begin : key_gen

		key_filter #(

			.CNT_MAX ( 20'd1000_000)

		) key_filter_inst (

			.sys_clk	(clk_50m),
			.sys_rst_n	(rst_n),
			.key_in		(key_sw[i]),

			.key_flag	(swing_key[i])

		);

		end
	endgenerate


//===========================================================================
// LED 指示: led[1:0] 显示控制状态, led[3:2] 显示按键(SW3/SW4)是否被检测到
//   状态: 全灭=空闲   led0亮=起摆中   led1亮=已平衡
//===========================================================================
led_ctrl led_ctrl_inst(

	.clk_50m	(clk_50m		),
	.rst_n		(rst_n			),

	.key_flag	(swing_key		),	// 来自 key_filter 的消抖按键脉冲
	.ctrl_state	(ctrl_state		),	// 来自 pendulum_ctrl 的状态输出(0空闲/1起摆/2平衡)
	.led		(led			)	// 送到顶层 led 端口 → 板载LED

);


//===========================================================================
// ADC 角度采集: 读角度传感器的10bit并行数据, 输出12bit摆杆角度 theta_out
//===========================================================================
wire 			[11:0]	theta_out	;
wire                    theta_sample_valid;
wire [11:0] theta_clean;

adc #(.TRIM_ENABLE(ADC_TRIM_ENABLE)) adc_inst(

   .clk_50m     (clk_50m	),
   .rst_n		(rst_n		),
   // 3PA1030硬件接口
   .adc_clk		(adc_clk	),	// ADC采样时钟输出
   .adc_oe_n	(adc_oe_n	),	// 输出使能低有效
   .adc_stby_n	(adc_stby_n	),	// 休眠低有效
   .adc_data_in	(adc_data_in),	// D9~D0并行数据
   .theta_out   (theta_out	),
   .sample_valid(theta_sample_valid),
   .theta_clean(theta_clean)

);

//===========================================================================
// 编码器: 读 XH2 电机的编码器 A/B 相, 输出累计位置计数 enc_pos1
//===========================================================================
	wire	[15:0]	enc_pos1	;

	encoder encoder_1 (

		.clk_50m		(clk_50m),
		.rst_n			(rst_n),

		.enc_a			(enc1_a),
		.enc_b			(enc1_b),

		.pos			(enc_pos1)	//编码器采集的位置环

	);


//===========================================================================
// 第二路编码器(XH1): 备用接口，不参与位置反馈或转臂串口观测。
wire	[15:0]	enc_pos2	;
encoder encoder_2 (

	.clk_50m		(clk_50m),
	.rst_n			(rst_n),

	.enc_a			(enc2_a),
	.enc_b			(enc2_b),

	.pos			(enc_pos2)	//编码器采集的位置(转臂)

);


//===========================================================================
// ADC: 每3ms窗口接受60个转换(50us间隔) -> 去6+6极值 -> 48点均值；
// 有条件3点中值去毛刺 -> 2点平均；下一时钟执行控制。
// 窗口仍为3ms、sample_valid相位不变，不增加控制延迟；平均点数从12提高到48，
// 转换器白噪声约降2倍，同时单窗口可剔除两侧各6个异常转换。
// 平滑运动仍保留原2点平均的1.5ms群延迟；大阶跃额外确认一拍(3ms)。
// 编码器位置直接反馈，微分在控制器中滤波，避免无符号均值的回绕突跳。
//===========================================================================
wire	[11:0]	theta_filt	;	// 角度滤波后(12bit)
wire	[15:0]	arm_pos_filt	;	// 编码器位置滤波后(16bit)

adc_spike_filter #(.WIDTH(12), .SPIKE_THRESHOLD(ADC_SPIKE_THRESHOLD)) theta_avg (
	.clk_50m	(clk_50m	),
	.rst_n		(rst_n		),
	.din		(theta_clean	),
	.sample_valid(theta_sample_valid),
	.dout		(theta_filt	)
);

assign arm_pos_filt = enc_pos1;


//===========================================================================
// 倒立摆控制器: SW4 触发自动起摆 + 倒立 PID 平衡 + 位置/轨迹控制
//   输入: theta_out(摆杆角度), enc_pos1(转臂位置), swing_key[3:0](按键)
//   输出: motor_sel(方向), motor_duty(占空比), ctrl_state(状态)
//===========================================================================
wire	[1:0]	motor_sel	;
wire	[7:0]	motor_duty_comb	;
reg		[7:0]	motor_duty	;
// 占空比打一拍再送 PWM/DAC/遥测：只延迟 20 ns（PWM 周期 40 us），
// 但把“控制律 → 占空比 → PWM/DAC/UART”这条最长的组合链切断。
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) motor_duty <= 8'd0;
    else       motor_duty <= motor_duty_comb;
end
wire    [15:0]  debug_out;
wire [5:0] trace_state;
wire trace_manual;
wire [15:0] trace_flags;
wire signed [15:0] trace_location, trace_pos_target, trace_u_duty;
wire signed [31:0] trace_angle_target_q8, trace_pos_output_q8, trace_angle_error_q8;
wire signed [31:0] trace_d_angle_q8, trace_d_pos_q8, trace_angle_integral_q8, trace_pos_integral_q8;
wire [223:0] trace_config;
wire usb_command_valid,usb_error_valid;
wire [15:0] usb_sequence,pid_command_sequence,pid_active_sequence;
wire [7:0] usb_operation,usb_bank,usb_error_result,pid_command_result,pid_command_bank;
wire [95:0] usb_gains,pid_readback_gains;
// Position commands (USB op4/op5) are accepted only while the controller is in
// the balance state; the receiver enforces this before forwarding the command.
wire usb_balancing = (trace_state == 6'd4);
usb_pid_command usb_pid_inst(
    .clk_50m(clk_50m),.rst_n(rst_n),.rx(uart_rx),.balancing(usb_balancing),
    .command_valid(usb_command_valid),.error_valid(usb_error_valid),
    .command_sequence(usb_sequence),.command_operation(usb_operation),.command_bank(usb_bank),
    .command_gains(usb_gains),.error_result(usb_error_result));
wire signed [15:0] trace_requested_pwm = motor_sel==2'b01 ? $signed({8'b0,motor_duty}) :
    motor_sel==2'b10 ? -$signed({8'b0,motor_duty}) : 16'sd0;

// PID 参数写入总线(蓝牙串口命令 -> pendulum_ctrl)
wire	[2:0]	param_type	;
wire	[15:0]	param_val	;
wire			param_wr	;

pid_cmd pid_cmd_inst(
	.clk_50m	(clk_50m	),
	.rst_n		(rst_n		),
	.rx			(hc04_rx	),
	.param_type	(param_type	),
	.param_val	(param_val	),
	.param_wr	(param_wr	)
);

pendulum_ctrl #(.ANGLE_D_HYST_COUNTS(ANGLE_D_HYST_COUNTS),
    .FRICTION_FF_DEFAULT(FRICTION_FF_DEFAULT),
    .ANGLE_D_FILTER_SHIFT(ANGLE_D_FILTER_SHIFT),
    .ANGLE_D_TWO_POLE(ANGLE_D_TWO_POLE),
    .FRICTION_FF_SOFT_ENABLE(FRICTION_FF_SOFT_ENABLE),
    .FRICTION_FF_POS_SCALE_Q8(FRICTION_FF_POS_SCALE_Q8),
    .FRICTION_FF_NEG_SCALE_Q8(FRICTION_FF_NEG_SCALE_Q8),
    .AUTO_KI_RAMP_ENABLE(1'b0)) pendulum_ctrl_inst(

	.clk_50m	(clk_50m	),
	.rst_n		(rst_n		),
	.start		(swing_key[3]),	// SW4 按下触发起摆, 再按一次停止
	.sw1		(swing_key[0]),	// SW1 切换手动起摆模式
	.sw2		(swing_key[1]),	// SW2 切换自动起摆模式
	.sw3		(swing_key[2]),	// SW3 按键
	.dip1		(~dip_sw1),		// 拨码1: OFF(高)=顺时针, ON(低)=逆时针

	.theta		(theta_filt	),	// 摆杆角度(ADC, 滤波后)
	.arm_pos	(arm_pos_filt	),	// 臂角度(编码器, 滤波后)

	.param_type	(param_type	),	// PID 参数类型(蓝牙串口)
	.param_val	(param_val	),	// PID 参数值(蓝牙串口)
	.param_wr	(param_wr	),	// PID 参数写脉冲(蓝牙串口)
    .usb_command_valid(usb_command_valid),.usb_error_valid(usb_error_valid),
    .usb_sequence(usb_sequence),.usb_operation(usb_operation),.usb_bank(usb_bank),
    .usb_error_result(usb_error_result),.usb_gains(usb_gains),
    .pid_command_sequence(pid_command_sequence),.pid_active_sequence(pid_active_sequence),
    .pid_command_result(pid_command_result),.pid_command_bank(pid_command_bank),
    .pid_readback_gains(pid_readback_gains),

	.motor_sel	(motor_sel	),
	.duty		(motor_duty_comb),
	.state_out	(ctrl_state	),
	.debug_out	(debug_out	),
    .trace_state(trace_state),.trace_manual(trace_manual),.trace_flags(trace_flags),
    .trace_location(trace_location),.trace_pos_target(trace_pos_target),
    .trace_angle_target_q8(trace_angle_target_q8),.trace_pos_output_q8(trace_pos_output_q8),
    .trace_angle_error_q8(trace_angle_error_q8),.trace_d_angle_q8(trace_d_angle_q8),
    .trace_d_pos_q8(trace_d_pos_q8),.trace_u_duty(trace_u_duty),
    .trace_angle_integral_q8(trace_angle_integral_q8),.trace_pos_integral_q8(trace_pos_integral_q8),
    .trace_config(trace_config)
);


//===========================================================================
// 调试输出: 当前为位置环输出 u_pos_t, 16bit 无符号(raw低16位)
//===========================================================================


//===========================================================================
// 电机A驱动: PWM A接口驱动转臂，使用倒立摆控制信号。
//===========================================================================
motor_ctrl motor_ctrl_A(

    .clk_50m		(clk_50m		),	//时钟输入
    .rst_n			(rst_n			),	//复位输入
    // 电机A 控制指令(来自 pendulum_ctrl)
    .motor_a_sel	(motor_sel		),
    .duty_out		(motor_duty		),	//duty_out
	//output
    .ain1			(AN1			),
    .ain2			(AN2			),

	.pwm			(PWMA			)

    );

//===========================================================================
// 电机B驱动: 备用接口，固定停止。
//===========================================================================
motor_ctrl motor_ctrl_B(

    .clk_50m		(clk_50m		),	//时钟输入
    .rst_n			(rst_n			),	//复位输入
    // 电机B 不使用, 停止
    .motor_a_sel	(2'b00			),
    .duty_out		(8'd0			),	//duty_out
	//output
    .ain1			(BN1			),
    .ain2			(BN2			),

	.pwm			(PWMB			)

    );

//===========================================================================
// 串口: ASCII 十进制文本输出, 普通串口调试助手(文本模式)直接可读
//   每 0.5s 发一行: "P±xxxxx Ayyyyy Ttttt\r\n"
//   P = 位置环输出 u_pos_t(有符号)   A = 编码器位置(无符号)   T = 摆杆角度 theta
//===========================================================================
wire legacy_debug_tx;
dbg_uart dbg_uart_inst (

	.clk_50m	(clk_50m		),
	.rst_n		(rst_n			),

	.pos		(debug_out		),	//位置环输出 u_pos_t(有符号)
	.arm		(arm_pos_filt	),	//编码器位置(XH2)
	.theta		(theta_filt		),	//摆杆角度(ADC, 滤波后)

	.tx			(legacy_debug_tx)

);

// USB记录口：115200波特100Hz；蓝牙仍使用原来的9600波特文本。
assign hc04_tx = legacy_debug_tx;
pendulum_trace_uart #(.FIRMWARE_ID(FRICTION_FF_SOFT_ENABLE ? 32'h26102402 : 32'h26102401)) trace_uart_inst(
    .clk_50m(clk_50m),.rst_n(rst_n),.state(trace_state),.manual_mode(trace_manual),
    .flags(trace_flags),.arm(arm_pos_filt),.location(trace_location),.pos_target(trace_pos_target),
    .theta_raw(theta_out),.theta(theta_filt),.angle_target_q8(trace_angle_target_q8),
    .pos_output_q8(trace_pos_output_q8),.angle_error_q8(trace_angle_error_q8),
    .d_angle_q8(trace_d_angle_q8),.d_pos_q8(trace_d_pos_q8),.u_duty(trace_u_duty),
    .requested_pwm(trace_requested_pwm),.angle_integral_q8(trace_angle_integral_q8),
    .pos_integral_q8(trace_pos_integral_q8),.config_words(trace_config),
    .pid_command_sequence(pid_command_sequence),.pid_active_sequence(pid_active_sequence),
    .pid_command_result(pid_command_result),.pid_command_bank(pid_command_bank),
    .pid_readback_gains(pid_readback_gains),.tx(uart_tx));

endmodule
