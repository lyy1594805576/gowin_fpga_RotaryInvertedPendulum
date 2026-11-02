`timescale 1ns/1ns
//===========================================================================
// pendulum_ctrl.v — 旋转倒立摆(Furuta) 共振自动启摆 + 串级 PID 倒立平衡
//---------------------------------------------------------------------------
// 算法移植自 STM32《16-倒立摆-自动启摆》(User/main.c + User/PID.c)。
// 原来的"能量起摆 + 单一并联 PID"已改为 STM32 的"峰值检测共振起摆 + 串级 PID"。
//
// 状态机(与 STM32 的 RunState 一一对应):
//   0     : 停止, 电机停转
//   1     : 判断(每隔 ~40ms 采样一次摆杆角度, 用连续 3 次采样判断摆杆位置)
//   21~24 : 向左施加瞬时驱动力  (+START_PWM -> 延时 -> -START_PWM -> 延时)
//   31~34 : 向右施加瞬时驱动力  (-START_PWM -> 延时 -> +START_PWM -> 延时)
//   4     : 串级 PID 倒立平衡(内环角度环 3ms + 外环位置环 50ms)
//
// 共振启摆原理(与 STM32 一致, 不是能量法):
//   摆杆摆到一侧最高点时(该侧角度为极小/极大值), 施加一次同向瞬时驱动力,
//   逐步把摆杆"泵"起来; 当摆杆进入中心区间后切入串级 PID 平衡。
//
// 串级 PID(与 STM32 一致):
//   内环角度环: out = Kp*e + Ki*∫e + Kd*(-测量值变化量，经低通), e = 目标角度 - 实际角度
//               out 直接给电机 PWM
//   外环位置环: out = Kp*e + Ki*∫e + Kd*(-测量值变化量，经低通), e = 目标位置 - 实际位置
//               out 作用于内环目标: 目标角度 = CENTER_ANGLE - 位置环out
//
// 硬件接口(ADC有效脉冲驱动滤波，控制节拍延后两时钟):
//   theta[11:0]  : 摆杆角度(12bit, 0~4092), 来自 adc 模块 theta_out
//   arm_pos[15:0]: 转臂位置(编码器 4 倍频计数, 1040 计/圈), 来自 encoder
//   motor_sel/duty: 电机方向/占空比, 送 top 的 motor_sel → motor_ctrl_A
//   按键: SW4(start)=开始/停止  SW1=手动模式  SW2=自动模式  SW3=平衡时顺时针转90度
//   USB op4/op5: 转臂绝对位置定位/保持(梯形速度曲线，见 goto_* 寄存器)
//===========================================================================
module pendulum_ctrl #(
    // ---- 摆杆角度标定 (对应 STM32 的 CENTER_ANGLE / CENTER_RANGE, 单位 12bit) ----
    // ---- 平衡点 (手动/自动各自独立) ----
    parameter [11:0] ANGLE_CENTER_MAN  = 12'd2957,  // 手动起摆: 倒立平衡点 theta 值(已标定, 对齐pid_control)
    parameter [11:0] ANGLE_CENTER_AUTO = 12'd2965,  // 自动起摆: 倒立平衡点 theta 值(已标定, 对齐pid_control)
    // ---- 中心区间半宽 (手动/自动各自独立) ----
    parameter [11:0] CENTER_RANGE_MAN  = 12'd400,   // 手动模式平衡: 摆杆进入 ANGLE_CENTER±此值 才算倒立
    parameter [11:0] CENTER_RANGE_AUTO = 12'd500,   // 自动模式: 峰值检测 + 平衡 共用的中心区间半宽

    // ---- 启摆(共振启摆)参数 (对应 STM32 的 START_PWM / START_TIME) ----
    parameter signed [8:0] SW_PWM  = 9'sd180,   // 恢复10:38成功记录的力度；增大冲量也会改变捕获轨迹，需单独复测
    parameter [7:0]        SW_TIME = 8'd65,     // 冲量正向(加速)段时长: 60 拍 ≈ 180ms
    parameter [7:0]        SW_TIME_RET = 8'd95, // 冲量反向(回转)段时长: 90 拍 ≈ 270ms, 比正向更长(回转行程不足, 加长)

    // ---- 启动模式默认值 (上电/复位后的模式; 运行时 SW1=手动 / SW2=自动 实时切换) ----
    parameter MANUAL_START_DEF = 1'b0,   // 上电默认: 1=手动起摆(手扶平衡点按SW4直接进PID) 0=自动共振启摆

    // ---- 采样分频(每拍 = 3ms) ----
    parameter [7:0] JUDGE_DIV = 8'd13,          // 判断采样 40ms(40/3≈13)
    parameter [7:0] POS_DIV   = 8'd17,         // 位置环 50ms(50/3≈17), 比角度环(3ms)慢很多, 避免打架

    // ---- 转臂位置 (对应 STM32 编码器 408=一圈; 本板实测 1063=一圈) ----
    // 实测标定：手动转一整圈物理 360°，编码器按旧 1040 换算显示约 368°，
    // 因此实际计数/圈 = 1040*368/360 ≈ 1063（2026-10-18）。
    parameter signed [15:0] ARM_REV     = 16'sd1063,  // 一圈编码器计数（实测标定）
    parameter signed [15:0] ARM_REV_MAX = 16'sd10630, // 位置目标 ±10 圈 (10×1063)
    parameter signed [1:0]  ENC_DIR     = -2'sd1,     // 编码器方向: 平衡后转臂转了一整圈(位置环反向) → 已改成 -1; 若改后反向转, 再改回 +1
    // PWM A/XH2已能自动倒立的pid_control使用-1。只作用于平衡PID输出，
    // 共振起摆的正反向冲量不乘此项；ENC_DIR是独立的位置反馈方向。
    parameter signed [1:0]  ANGLE_DIR   = -2'sd1,

    // ---- 手动起摆 PID，沿用当前标定参数 ----
    //   Q12 定点(实际增益 = 参数/4096)
    parameter signed [15:0] KP_A_MAN = 16'sd1750,
    parameter signed [15:0] KI_A_MAN = 16'sd47,
    parameter signed [15:0] KD_A_MAN = 16'sd24890,
    parameter signed [15:0] KP_P_MAN = 16'sd1400,
    parameter signed [15:0] KI_P_MAN = 16'sd10,
    parameter signed [15:0] KD_P_MAN = 16'sd5509,

    // ---- 自动起摆 PID，捕获后持续使用设定的六项增益 ----
    parameter signed [15:0] KP_A_AUTO = 16'sd1750,
    parameter signed [15:0] KI_A_AUTO = 16'sd47,
    parameter signed [15:0] KD_A_AUTO = 16'sd27000,
    parameter signed [15:0] KP_P_AUTO = 16'sd1400,
    parameter signed [15:0] KI_P_AUTO = 16'sd8,
    parameter signed [15:0] KD_P_AUTO = 16'sd5509,

    // ---- 误差死区 (手动/自动各自独立, 忽略微小误差) ----
    parameter signed [15:0] ANGLE_DB_MAN  = 16'sd3,   // 手动: 角度环误差死区
    parameter signed [15:0] POS_DB_MAN    = 16'sd3,  // 手动: 位置环误差死区
    parameter signed [15:0] ANGLE_DB_AUTO = 16'sd3,   // 自动: 对齐已验证pid_control参考, 抑制LSB噪声经P/D放大造成的抖动
    parameter signed [15:0] POS_DB_AUTO   = 16'sd1,  // 自动: 恢复成功记录的位置死区

    // ---- 积分限幅 (手动/自动各自独立, anti-windup) ----
    parameter signed [31:0] ANGLE_INT_LIM_MAN  = 32'sd10000,  // 手动: 角度环积分限幅
    parameter signed [31:0] POS_INT_LIM_MAN    = 32'sd5000,   // 手动: 位置环积分限幅
    parameter signed [31:0] ANGLE_INT_LIM_AUTO = 32'sd10000,  // 自动: 角度环积分限幅
    parameter signed [31:0] POS_INT_LIM_AUTO   = 32'sd5000,   // 自动: 位置环积分限幅

    // ---- 输出限幅 (手动/自动各自独立) ----
    parameter signed [15:0] OUT_MAX_MAN      = 16'sd100,  // 手动: 角度环输出限幅
    parameter signed [15:0] POS_OUT_MAX_MAN  = 16'sd300,  // 手动: 位置环输出限幅
    parameter signed [15:0] OUT_MAX_AUTO     = 16'sd100,  // 自动: 角度环输出限幅
    parameter signed [15:0] POS_OUT_MAX_AUTO = 16'sd600,  // 自动: 位置环输出限幅

    // ---- 占空比转换(固定换算, 两模式共用) / 电机死区(手动/自动各自独立) ----
    parameter signed [15:0] DUTY_SCALE   = 16'sd2611, // 255/100*1024≈2611, >>10 把 ±100 转为 ±255 占空比
    parameter signed [8:0]  DEADZONE_MAN  = 9'sd3,     // 手动: 电机输出死区(占空比)
    parameter signed [8:0]  DEADZONE_AUTO = 9'sd3,     // 自动: 电机输出死区(占空比)
    // Smooth actuator friction inverse, PWM counts (0=baseline, USB op7:0..12).
    // Leave off until an on-board A/B trial; no motor-friction identification
    // has been performed. Does not change PID gains or create drive at rest.
    parameter [3:0] FRICTION_FF_DEFAULT = 4'd0,
    parameter FRICTION_FF_SOFT_ENABLE = 1,
    // PWM polarity scales, 0..256=0..100%. Equal until independently measured.
    parameter [8:0] FRICTION_FF_POS_SCALE_Q8 = 9'd256,
    parameter [8:0] FRICTION_FF_NEG_SCALE_Q8 = 9'd256,

    // ---- SW3 顺时针旋转 90° 控制 ----
    parameter signed [15:0] ROT_90   = 16'sd266,  // 90° = 一圈(1063)/4 ≈266 编码器计数; 若方向反了改成 -266
    parameter signed [15:0] ROT_STEP = 16'sd8,     // 每个位置环(50ms)逼近计数, 越小越慢; 8→约1.6s转完90°
    // op4 position-move lead governor: while the arm is farther than this many
    // encoder counts behind the reference, the reference pauses so the arm can
    // catch up. This bounds tracking lag and therefore the overshoot after the
    // target stops. 0 disables the governor (pure profile). 64 (2026-10-18
    // speed build) doubles the tracking-lag authority vs 32 for faster moves.
    parameter [15:0] POS_LEAD_MAX = 16'd64,
    // Final-approach gate (0x26101806): the last position tick only completes
    // when the arm is within POS_FINAL_GATE counts of the reference AND its
    // measured position speed is below POS_FINAL_SPEED_Q8 (counts per 50 ms,
    // Q8). This stops the arm coasting at speed when the reference stops.
    // 0 disables the gate (finish as soon as the reference distance is covered).
    parameter [15:0] POS_FINAL_GATE = 16'd8,
    parameter signed [15:0] POS_FINAL_SPEED_Q8 = 16'sd512,
    // Predictive stop gate (0x26101807): additionally require that the
    // measured arm velocity cannot coast past the target:
    //   coast = (velocity toward target in counts/tick * POS_STOP_COAST_TICKS)
    // and completion needs |lead| + coast <= POS_STOP_LIMIT counts.
    // POS_STOP_LIMIT=0 disables this extension (0x26101806 behaviour).
    parameter [15:0] POS_STOP_LIMIT = 16'd6,
    parameter [3:0] POS_STOP_COAST_TICKS = 4'd8,
    // 控制周期不变，ADC→滤波→控制依次相隔一个50MHz时钟。
    parameter [17:0] TICK_CNT_MAX = 18'd149999,
    parameter [17:0] TICK_PHASE = 18'd1,
    // Measurement hysteresis for the angle derivative only, in expanded ADC
    // counts. Zero preserves the original derivative exactly; optional trial.
    parameter [11:0] ANGLE_D_HYST_COUNTS = 12'd0,
    // D-only IIR: alpha = 1/2^shift. 1 preserves the historical half filter;
    // production top selects 2 (alpha=1/4). Valid settings: 1..3.
    // P/I, position feedback, capture/fall detection and ADC timing are separate.
    parameter ANGLE_D_FILTER_SHIFT = 1,
    // Opt-in two-pole D low-pass; both poles alpha=5/16, internal Q16.
    parameter ANGLE_D_TWO_POLE = 0,
    // 外环角度指令每3ms最多变化2个ADC扩展计数，保留Q8小数。
    parameter signed [31:0] TARGET_STEP_Q8 = 32'sd512,
    // Historical KI-decay experiment is opt-in only. With KI_A_AUTO=60 it
    // otherwise reduces the configured gain to30 after18.03s and6 after30.054s.
    // Production must keep configured gains after capture, including defaults
    // restored over USB. Explicit UART gain changes remain supported.
    parameter AUTO_KI_RAMP_ENABLE = 1'b0,
    parameter [15:0] AUTO_KI_CAPTURE_TICKS = 16'd1000,
    parameter [15:0] AUTO_KI_RAMP_DIV = 16'd167,
    parameter signed [15:0] KI_A_AUTO_STEADY = 16'sd6
)(
    input             clk_50m,      // 50MHz时钟, 来自 top 的 clk_50m
    input             rst_n,        // 复位(低有效), 来自 top 的 rst_n
    input             start,        // SW4 起摆/停止触发(按下一次), 来自 top 的 swing_key[3]
    input             sw1,          // SW1 切手动模式触发, 来自 top 的 swing_key[0]
    input             sw2,          // SW2 切自动模式触发, 来自 top 的 swing_key[1]
    input             sw3,          // SW3 按键, 来自 top 的 swing_key[2]
    input             dip1,         // 拨码开关1: 0=顺时针 1=逆时针
    input  [11:0]     theta,        // 摆杆角度(12bit), 来自 top 的 theta_out(adc 模块输出)
    input  [15:0]     arm_pos,      // 转臂位置(编码器计数), 来自 top 的 enc_pos1(encoder 模块输出)
    input  [2:0]      param_type,   // PID 参数类型 0~5, 来自 pid_cmd (蓝牙串口命令)
    input  [15:0]     param_val,    // PID 参数值(有符号), 来自 pid_cmd
    input             param_wr,     // PID 参数写脉冲, 来自 pid_cmd
    input             usb_command_valid, usb_error_valid,
    input [15:0]      usb_sequence,
    input [7:0]       usb_operation, usb_bank, usb_error_result,
    input [95:0]      usb_gains,
    output reg [15:0] pid_command_sequence, pid_active_sequence,
    output reg [7:0]  pid_command_result, pid_command_bank,
    output [95:0]     pid_readback_gains,
    output reg [1:0]  motor_sel,    // 电机方向 00停 01正转 10反转, 送给 top 的 motor_sel → motor_ctrl_A
    output reg [7:0]  duty,         // 电机PWM占空比 0~255, 送给 top 的 motor_duty → motor_ctrl_A
    output reg [1:0]  state_out,    // 调试状态 0停止 1起摆 2平衡, 送给 top 的 ctrl_state → led_ctrl
    output reg [15:0] debug_out,    // 旧ASCII调试输出
    // 观测端口；USB命令之外的控制状态机和标定保持原工程行为。
    output [5:0] trace_state,
    output trace_manual,
    output [15:0] trace_flags,
    output signed [15:0] trace_location, trace_pos_target,
    output signed [31:0] trace_angle_target_q8, trace_pos_output_q8,
    output signed [31:0] trace_angle_error_q8, trace_d_angle_q8, trace_d_pos_q8,
    output signed [15:0] trace_u_duty,
    output signed [31:0] trace_angle_integral_q8, trace_pos_integral_q8,
    output [223:0] trace_config
);

// 状态编码保留原工程：停止、共振起摆、串级平衡。
localparam [5:0] S_STOP=6'd0, S_JUDGE=6'd1, S_BAL=6'd4;
localparam [5:0] S_L1=6'd21, S_L2=6'd22, S_L3=6'd23, S_L4=6'd24;
localparam [5:0] S_R1=6'd31, S_R2=6'd32, S_R3=6'd33, S_R4=6'd34;
localparam [3:0] FALL_CNT_MAX=4'd9;
reg [5:0] run_state;
reg manual_mode;
reg [17:0] tick_cnt;
wire tick = (tick_cnt == TICK_PHASE);
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) tick_cnt <= 0;
    else if(tick_cnt == TICK_CNT_MAX) tick_cnt <= 0;
    else tick_cnt <= tick_cnt+1'b1;
end
reg [7:0] judge_cnt, pos_cnt, count_time;
wire judge_tick = (judge_cnt == JUDGE_DIV-1'b1);
wire pos_tick = (pos_cnt == POS_DIV-1'b1);
reg [3:0] key_d1, key_pend;
wire [3:0] key_edge = {start,sw3,sw2,sw1} & ~key_d1;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin key_d1 <= 0; key_pend <= 0; end
    else begin
        key_d1 <= {start,sw3,sw2,sw1};
        if(tick) key_pend <= key_edge;
        else key_pend <= key_pend | key_edge;
    end
end
wire requested_mode = manual_mode;   // SW1/SW2 改为旋转按键，模式只由板级锁存/上位机决定
wire mode_change = tick && (requested_mode != manual_mode);
reg signed [15:0] kp_a_man_r, ki_a_man_r, kd_a_man_r, kp_p_man_r, ki_p_man_r, kd_p_man_r;
reg signed [15:0] kp_a_auto_r, ki_a_auto_r, kd_a_auto_r, kp_p_auto_r, ki_p_auto_r, kd_p_auto_r;
reg signed [15:0] ki_a_auto_balance_r;
reg [15:0] auto_capture_cnt, auto_ki_ramp_cnt;
reg auto_ki_override;
reg usb_pending;
reg [15:0] pending_sequence;
reg [7:0] pending_operation;
reg pending_bank;
reg [95:0] pending_gains;
reg [3:0] friction_ff_ks;
wire pending_friction_ff = pending_operation==8'd7;
// Position commands (USB op4/op5) reuse the 96-bit command payload:
// [31:0] target encoder counts, [47:32] slew step per 50 ms position tick.
wire signed [31:0] pending_target = $signed(pending_gains[31:0]);
wire signed [15:0] pending_step = $signed(pending_gains[47:32]);
wire pending_position = (pending_operation==8'd4) || (pending_operation==8'd5);
wire signed [15:0] pending_target_clamped = (pending_target > ARM_REV_MAX) ? ARM_REV_MAX :
    (pending_target < -ARM_REV_MAX) ? -ARM_REV_MAX : pending_target[15:0];
// Commit at the END of a control period, before the next PID start. Both
// pipelined PID instances therefore latch one coherent set of six gains.
wire usb_commit = usb_pending && tick_cnt == TICK_CNT_MAX;
wire [95:0] default_auto_gains={KD_P_AUTO,KI_P_AUTO,KP_P_AUTO,KD_A_AUTO,KI_A_AUTO,KP_A_AUTO};
wire [95:0] default_man_gains={KD_P_MAN,KI_P_MAN,KP_P_MAN,KD_A_MAN,KI_A_MAN,KP_A_MAN};
// Bank 0x80 tags an op7 readback: word0=actual friction KS, words1..5=0.
// Normal PID readback keeps banks0/1 and its original six-word meaning.
assign pid_readback_gains=pid_command_bank==8'h80 ? {92'd0,friction_ff_ks} : pid_command_bank==1 ?
    {kd_p_man_r,ki_p_man_r,kp_p_man_r,kd_a_man_r,ki_a_man_r,kp_a_man_r} :
    {kd_p_auto_r,ki_p_auto_r,kp_p_auto_r,kd_a_auto_r,ki_a_auto_r,kp_a_auto_r};
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        usb_pending<=0;pending_sequence<=0;pending_operation<=0;pending_bank<=0;pending_gains<=0;
        pid_command_sequence<=0;pid_active_sequence<=0;pid_command_result<=0;
        pid_command_bank<=MANUAL_START_DEF ? 8'd1 : 8'd0;
    end else begin
        if(usb_commit) begin
            usb_pending<=0;pid_active_sequence<=pending_sequence;pid_command_sequence<=pending_sequence;
            pid_command_bank<=pending_friction_ff ? 8'h80 : {7'd0,pending_bank};
            // Position commands only take effect while balancing; the actual
            // rot_goal / pos_slew_step registers are written in the state-machine
            // block below so each register keeps a single driver.
            if(pending_position)
                pid_command_result <= (run_state==S_BAL) ? 8'd2 : 8'd8;
            else
                pid_command_result<=2;
        end
        // A pending set cannot be replaced by a later command, even on commit.
        if(usb_command_valid) begin
            if(!usb_pending) begin
                pid_command_sequence<=usb_sequence;
                pid_command_bank<=usb_operation==7 ? 8'h80 : usb_bank;
                if(usb_operation==2 || (usb_operation==7 && usb_gains[15:0]==16'hffff)) pid_command_result<=7;
                else begin
                    pending_sequence<=usb_sequence;pending_operation<=usb_operation;pending_bank<=usb_bank[0];
                    pending_gains<=usb_operation==3 ? (usb_bank[0] ? default_man_gains : default_auto_gains) : usb_gains;
                    usb_pending<=1;pid_command_result<=1;
                end
            end
        end else if(usb_error_valid && !usb_pending) begin
            pid_command_sequence<=usb_sequence;pid_command_bank<=usb_bank;pid_command_result<=usb_error_result;
        end
    end
end
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) friction_ff_ks <= FRICTION_FF_DEFAULT;
    else if(usb_commit && pending_friction_ff) friction_ff_ks <= pending_gains[3:0];
    else if(usb_commit && pending_operation==3) friction_ff_ks <= FRICTION_FF_DEFAULT;
end
wire signed [15:0] KP_A_ACT = manual_mode ? kp_a_man_r : kp_a_auto_r;
wire auto_ki_scheduled = AUTO_KI_RAMP_ENABLE && !auto_ki_override && !manual_mode;
wire signed [15:0] KI_A_ACT = manual_mode ? ki_a_man_r :
    auto_ki_scheduled ? ki_a_auto_balance_r : ki_a_auto_r;
wire signed [15:0] KD_A_ACT = manual_mode ? kd_a_man_r : kd_a_auto_r;
wire signed [15:0] KP_P_ACT = manual_mode ? kp_p_man_r : kp_p_auto_r;
// 运动/静止两套位置环：定位（含摇杆）时位置环更硬，静止时用原值。
// 用移位实现，不引入乘法；KP 上限约 16000，×2 不会溢出常用范围。

wire signed [15:0] KI_P_ACT = manual_mode ? ki_p_man_r : ki_p_auto_r;
wire signed [15:0] KD_P_ACT = manual_mode ? kd_p_man_r : kd_p_auto_r;
reg [11:0] center_man_r, center_auto_r;   // op13 在线可调（复位回参数默认）
wire [11:0] ANGLE_CENTER_ACT = manual_mode ? center_man_r : center_auto_r;
wire [11:0] CENTER_RANGE_ACT = manual_mode ? CENTER_RANGE_MAN : CENTER_RANGE_AUTO;
wire signed [15:0] ANGLE_DB_ACT = manual_mode ? ANGLE_DB_MAN : ANGLE_DB_AUTO;
wire signed [15:0] POS_DB_ACT = manual_mode ? POS_DB_MAN : POS_DB_AUTO;
wire signed [31:0] ANGLE_INT_LIM_ACT = manual_mode ? ANGLE_INT_LIM_MAN : ANGLE_INT_LIM_AUTO;
wire signed [31:0] POS_INT_LIM_ACT = manual_mode ? POS_INT_LIM_MAN : POS_INT_LIM_AUTO;
wire signed [15:0] OUT_MAX_ACT = manual_mode ? OUT_MAX_MAN : OUT_MAX_AUTO;
wire signed [15:0] POS_OUT_MAX_ACT_RAW = manual_mode ? POS_OUT_MAX_MAN : POS_OUT_MAX_AUTO;
// 静止时位置环权限减半：600 在静止会顶出极限环（实测静止摆幅 20°），
// 运动时才放开到 600 拿刹车力。
wire signed [15:0] POS_OUT_MAX_ACT = motion_active ?
    POS_OUT_MAX_ACT_RAW : (POS_OUT_MAX_ACT_RAW >>> 1);
wire signed [8:0] DEADZONE_ACT = manual_mode ? DEADZONE_MAN : DEADZONE_AUTO;
// 中值/范围先算好打进寄存器，避免“寄存器选择 → 加减 → 比较 → 状态机”的长组合链
reg [11:0] CENTER_LO, CENTER_HI;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        CENTER_LO <= ANGLE_CENTER_AUTO-CENTER_RANGE_AUTO;
        CENTER_HI <= ANGLE_CENTER_AUTO+CENTER_RANGE_AUTO;
    end else begin
        CENTER_LO <= ANGLE_CENTER_ACT-CENTER_RANGE_ACT;
        CENTER_HI <= ANGLE_CENTER_ACT+CENTER_RANGE_ACT;
    end
end
wire theta_in_range = theta > CENTER_LO && theta < CENTER_HI;
wire signed [31:0] theta_s = $signed({20'b0,theta});
wire signed [31:0] center_s = $signed({20'b0,ANGLE_CENTER_ACT});
wire signed [31:0] requested_center = requested_mode ?
    $signed({20'b0,center_man_r}) : $signed({20'b0,center_auto_r});
reg [15:0] zero_ref;
// Subtract in 16 bits BEFORE sign extension. A wrap from FFFE to 0001 is +3.
// Relative travel remains inside +/-32768 counts (target limit is +/-10630 = 10 rev).
wire signed [15:0] location_mod = arm_pos-zero_ref;
wire signed [31:0] location = {{16{location_mod[15]}},location_mod};
reg signed [15:0] pos_target, rot_goal, pos_slew_step;
// Trapezoidal position-move profile for USB op4 (like jiexiev5's speed curve):
// speed starts at 1 count/50 ms tick, ramps +1/tick up to vmax_step, cruises,
// and brakes -(1/tick) once the remaining distance reaches the braking distance
// of the current speed. It stops exactly on the target without overshoot.
// op5 (hold) only freezes the current position target.
reg goto_active;
// 运动标志：goto_active（含摇杆）时用"运动中"那套参数
wire motion_active = goto_active;
wire signed [15:0] KP_P_EFF = KP_P_ACT;
wire signed [15:0] KI_P_EFF = KI_P_ACT;
reg signed [15:0] goto_target;
reg signed [15:0] vmax_step;
reg signed [15:0] move_dir;
reg [15:0] move_speed;
reg [15:0] rem_r;          // remaining distance registered for the current tick
reg goto_calc;             // one-cycle flag: re-compute rem/speed after a tick
wire signed [31:0] pos_target_s = {{16{pos_target[15]}},pos_target};
wire signed [31:0] rot_remain = $signed({{16{rot_goal[15]}},rot_goal})-pos_target_s;
reg signed [31:0] angle_target_q8, pos_output_q8;
reg signed [31:0] theta_prev, location_prev, d_angle_q8, d_pos_q8;
reg signed [31:0] theta_d_prev;
reg signed [31:0] d_stage1_q16, d_stage2_q16;
reg [1:0] d_phase;
reg signed [31:0] d_error_hold, d_int_limit_hold;
reg signed [15:0] d_kp_hold, d_ki_hold, d_kd_hold, d_out_limit_hold;
reg [11:0] ang_prev1, ang_prev2;
reg [3:0] fall_cnt;
reg signed [9:0] u_duty;
reg signed [47:0] duty_product;
reg duty_pending;
function signed [31:0] half_signed;
    input signed [31:0] x;
    begin half_signed = x < 0 ? -((-x) >>> 1) : (x >>> 1); end
endfunction
function signed [31:0] angle_d_divide;
    input signed [31:0] x;
    // Truncate symmetrically toward zero. Arithmetic shift alone would retain
    // a negative residue at rest, causing a direction-dependent PWM bias.
    begin angle_d_divide = x < 0 ? -((-x) >>> ANGLE_D_FILTER_SHIFT) :
        (x >>> ANGLE_D_FILTER_SHIFT); end
endfunction
function signed [31:0] divide16_signed;
    input signed [35:0] x;
    begin divide16_signed = x < 0 ? -((-x) >>> 4) : (x >>> 4); end
endfunction
function signed [31:0] q16_to_q8;
    input signed [31:0] x;
    reg [31:0] magnitude;
    begin
        magnitude = x[31] ? -x : x;
        q16_to_q8 = x[31] ? -$signed((magnitude+32'd128) >> 8) :
                                      $signed((magnitude+32'd128) >> 8);
    end
endfunction
function signed [31:0] round_q8;
    input signed [31:0] x;
    reg [31:0] magnitude;
    begin
        magnitude = x[31] ? -x : x;
        round_q8 = x[31] ? -$signed((magnitude+32'd128) >> 8) :
                                   $signed((magnitude+32'd128) >> 8);
    end
endfunction
function signed [31:0] round_duty;
    input signed [47:0] x;
    reg [47:0] magnitude, rounded;
    begin
        magnitude = x[47] ? -x : x;
        rounded = (magnitude+48'd131072) >> 18;
        round_duty = x[47] ? -$signed(rounded[31:0]) : $signed(rounded[31:0]);
    end
endfunction
// The goal is bounded once when a key command arrives. The trajectory then
// approaches that goal without cascaded 32-bit add/clamp/subtract operations.
localparam signed [15:0] ROT_180_K = 16'sd532;
localparam signed [15:0] ROT_360_K = 16'sd1064;
wire signed [15:0] key_span = key_pend[0] ? ROT_90 :
                              key_pend[1] ? ROT_180_K : ROT_360_K;
wire signed [15:0] key_span_s = dip1 ? -key_span : key_span;
wire signed [16:0] goal_sum = $signed({rot_goal[15],rot_goal})+
    $signed({key_span_s[15],key_span_s});
wire signed [15:0] next_goal = (goal_sum > ARM_REV_MAX) ? ARM_REV_MAX :
    (goal_sum < -ARM_REV_MAX) ? -ARM_REV_MAX : goal_sum[15:0];
wire signed [16:0] goal_delta = $signed({rot_goal[15],rot_goal})-
    $signed({pos_target[15],pos_target});
wire signed [15:0] next_pos_target = (goal_delta > pos_slew_step) ? pos_target+pos_slew_step :
    (goal_delta < -pos_slew_step) ? pos_target-pos_slew_step : rot_goal;
// ---- position-command velocity profile (op4) ----
// Two-phase datapath to keep the 50 MHz timing: the position tick only adds the
// current speed, and the cycle after the tick re-computes the remaining
// distance and the next speed from registered values. The profile decision for
// a tick uses the remaining distance registered FOR that tick, which matches
// the reference trapezoid model.
wire signed [16:0] cmd_delta = $signed({pending_target_clamped[15],pending_target_clamped})-
    $signed({pos_target[15],pos_target});
wire [16:0] cmd_mag = cmd_delta[16] ? -cmd_delta : cmd_delta;
// Narrow multipliers: speed is at most 64 (7 bits), so the braking distances
// fit in 12 bits; the 16x16 array multiplier was the critical path.
wire [7:0] speed_sum1 = {1'b0,move_speed[6:0]}+8'd1;
wire [7:0] speed_sum2 = speed_sum1+8'd1;
wire [13:0] brake_prod = {7'd0,move_speed[6:0]}*{7'd0,speed_sum1[6:0]};
wire [13:0] brake_next_prod = {7'd0,speed_sum1[6:0]}*{7'd0,speed_sum2[6:0]};
wire [15:0] speed_brake = {2'd0,brake_prod}>>1;
wire [15:0] speed_plus1 = {8'd0,speed_sum1};
wire [15:0] speed_next_brake = {2'd0,brake_next_prod}>>1;
wire [15:0] move_speed_next = (rem_r <= speed_brake) ?
        ((move_speed > 16'd1) ? move_speed-16'd1 : 16'd1) :
    ((move_speed < vmax_step && rem_r >= speed_next_brake+speed_plus1) ?
        speed_plus1 : move_speed);
// Execute the current speed first, then update it after the tick: the first
// position-tick step is exactly 1 count (smooth start).
wire [15:0] move_step = (rem_r < move_speed) ? rem_r : move_speed;
wire goto_arrive = (rem_r <= move_step);
wire signed [16:0] move_step_s = (move_dir < 0) ? -$signed({1'b0,move_step}) :
    $signed({1'b0,move_step});
wire signed [16:0] pos_target_adv = $signed({pos_target[15],pos_target})+move_step_s;
// Lead governor: signed distance by which the internal reference is ahead of
// the measured arm position in the direction of travel. When it reaches
// POS_LEAD_MAX the reference pauses until the arm catches up.
wire signed [31:0] lead_forward = (move_dir < 0) ? (location-pos_target_s) :
    (pos_target_s-location);
wire lead_hold = (POS_LEAD_MAX != 16'd0) &&
    (lead_forward >= $signed({16'd0,POS_LEAD_MAX}));
// Final-approach gate: finish only when the arm is close to the reference and
// moving slowly, so it cannot coast past the target after it stops. The
// predictive extension (0x26101807) additionally bounds the coasting distance
// derived from the current measured velocity.
wire [31:0] lead_abs = (lead_forward[31]) ? -lead_forward : lead_forward;
wire signed [31:0] dpos_abs = (d_pos_q8[31]) ? -d_pos_q8 : d_pos_q8;
wire signed [31:0] vel_toward = (move_dir < 0) ? -d_pos_q8 : d_pos_q8;
wire [15:0] vel_toward_pos = (vel_toward[31]) ? 16'd0 : vel_toward[15:0];
// Registered coast predictor (one cycle latency, inputs only change on the
// 50 ms position tick) keeps the 16x4 multiply off the state-machine path.
reg [15:0] coast_pred_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) coast_pred_r <= 16'd0;
    else coast_pred_r <= (vel_toward_pos * {12'd0,POS_STOP_COAST_TICKS}) >> 8;
end
wire stop_gate_ok = (POS_STOP_LIMIT == 16'd0) ? 1'b1 :
    ((lead_abs + {16'd0,coast_pred_r}) <= {16'd0,POS_STOP_LIMIT});
wire gate_cond = (POS_FINAL_GATE == 16'd0) ? 1'b1 :
    ((lead_abs <= {16'd0,POS_FINAL_GATE}) &&
     (dpos_abs <= $signed(POS_FINAL_SPEED_Q8)) &&
     stop_gate_ok);
// Register the complete gate decision so the comparison tree only feeds a
// flip-flop; the state-machine path stays short at 50 MHz.
reg final_gate_ok_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) final_gate_ok_r <= 1'b1;
    else final_gate_ok_r <= gate_cond;
end
wire final_gate_ok = final_gate_ok_r;
wire signed [31:0] angle_error_q8 = angle_target_q8-(theta_s <<< 8);
wire signed [31:0] angle_db_q8 = $signed(ANGLE_DB_ACT) <<< 8;
wire signed [31:0] angle_error_db_q8 = (angle_error_q8 > angle_db_q8) ?
    angle_error_q8-angle_db_q8 : (angle_error_q8 < -angle_db_q8) ?
    angle_error_q8+angle_db_q8 : 32'sd0;
// Use the current registered target. A key step takes effect on the next
// position sample, as in the original 51 ms outer loop.
wire signed [31:0] pos_error = (ENC_DIR < 0) ? location-pos_target_s : pos_target_s-location;
wire signed [31:0] pos_error_db = (pos_error > POS_DB_ACT) ? pos_error-POS_DB_ACT :
    (pos_error < -POS_DB_ACT) ? pos_error+POS_DB_ACT : 32'sd0;
// A tracking hysteresis removes tiny reversals before differentiating. Its
// anchor stays within +/- H of the measurement; sustained slow drift moves
// the anchor after H counts, so small per-tick motion is not lost forever.
// P/I error, capture and fall detection still use theta_s directly.
wire signed [31:0] angle_d_hyst = $signed({20'b0,ANGLE_D_HYST_COUNTS});
wire signed [31:0] theta_d_delta = theta_s-theta_d_prev;
wire signed [31:0] theta_d_next = theta_d_delta > angle_d_hyst ? theta_s-angle_d_hyst :
    (theta_d_delta < -angle_d_hyst ? theta_s+angle_d_hyst : theta_d_prev);
wire signed [31:0] angle_measurement_delta = ANGLE_D_HYST_COUNTS == 0 ?
    theta_prev-theta_s : theta_d_prev-theta_d_next;
// Keep derivative on measurement and retain Q8 sub-count velocity. Low-pass
// only D: d[n] = ((2^shift-1)*d[n-1] + delta_theta_q8)/2^shift.
// At 3 ms, shift=2 has ~15.4 Hz cutoff and 9 ms low-frequency group delay;
// shift=1 was ~38.3 Hz and 3 ms. No amplitude deadband or motion-dependent
// switching: slow/small real motion keeps the same steady derivative gain.
wire signed [31:0] angle_d_weighted = (d_angle_q8 <<< ANGLE_D_FILTER_SHIFT)-
    d_angle_q8+(angle_measurement_delta <<< 8);
wire signed [31:0] d_angle_legacy_next = angle_d_divide(angle_d_weighted);
wire signed [35:0] d_input_q16 = $signed(angle_measurement_delta) * 36'sd65536;
wire signed [35:0] d_first_weighted = $signed(d_stage1_q16)*36'sd11+d_input_q16*36'sd5;
wire signed [31:0] d_first_next = divide16_signed(d_first_weighted);
// Stage two consumes stage one's registered NEW sample on the next 50 MHz
// clock. Latch the original P/I error/gains to preserve the sampling instant.
wire signed [35:0] d_second_weighted = $signed(d_stage2_q16)*36'sd11+$signed(d_stage1_q16)*36'sd5;
wire signed [31:0] d_second_next = divide16_signed(d_second_weighted);
wire signed [31:0] d_angle_next = ANGLE_D_TWO_POLE ? q16_to_q8(d_stage2_q16) : d_angle_legacy_next;
wire signed [31:0] d_pos_next = half_signed(d_pos_q8+((ENC_DIR*(location_prev-location)) <<< 8));
// ---- 速度环 P（op15）：参考速度 − 实测速度，补到倾斜指令 ----
// 参考速度 = 梯形规划的 move_speed×move_dir（goto_active 时有效），
// 实测速度 = d_pos_q8 的整数部分。三级流水 + 余程分档淡出，0 = 逐位不变。
reg signed [15:0] vp_gain_q8;
// 加速度前馈：位置环不再靠"误差"去挤速度，加减速都由前馈直接给倾斜
reg signed [15:0] ff_gain_q8;
wire signed [15:0] vp_meas = $signed(d_pos_q8[23:8]);
// 参考速度：move_dir 是 ±1，用条件取反代替乘法，并打一拍切断组合链
wire signed [16:0] vp_ref_mag = $signed({1'b0,move_speed});
wire signed [16:0] vp_ref_signed = move_dir[15] ? -vp_ref_mag : vp_ref_mag;
reg signed [15:0] vp_ref_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) vp_ref_r <= 16'sd0;
    else       vp_ref_r <= goto_active ? vp_ref_signed[15:0] : 16'sd0;
end
wire signed [15:0] vp_ref = vp_ref_r;
wire signed [15:0] vp_err_raw = vp_ref - vp_meas;
reg signed [15:0] vp_err_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) vp_err_r <= 16'sd0;
    else if(run_state == S_BAL)
        vp_err_r <= (vp_err_raw > 16'sd256) ? 16'sd256 :
                    (vp_err_raw < -16'sd256) ? -16'sd256 : vp_err_raw;
    else vp_err_r <= 16'sd0;
end
reg signed [31:0] vp_prod_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) vp_prod_r <= 32'sd0;
    else vp_prod_r <= $signed(vp_gain_q8) * $signed(vp_err_r);
end
wire [2:0] vp_shift = (rem_r >= 16'd128) ? 3'd0 :
                      (rem_r >= 16'd64)  ? 3'd1 :
                      (rem_r >= 16'd32)  ? 3'd2 :
                      (rem_r >= 16'd16)  ? 3'd3 : 3'd4;
wire [2:0] vp_shift_act = goto_active ? vp_shift : 3'd0;
wire signed [31:0] vp_mid = (vp_prod_r >>> 4) >>> vp_shift_act;
wire signed [31:0] vp_hi = $signed({16'd0,POS_OUT_MAX_ACT});
reg signed [31:0] vp_term_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) vp_term_r <= 32'sd0;
    else if(vp_mid > vp_hi) vp_term_r <= vp_hi;
    else if(vp_mid < -vp_hi) vp_term_r <= -vp_hi;
    else vp_term_r <= vp_mid;
end
reg signed [15:0] move_speed_d;
reg signed [31:0] ff_accel_q8;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        move_speed_d <= 16'sd0;
        ff_accel_q8 <= 32'sd0;
    end else begin
        move_speed_d <= $signed({1'b0,move_speed});
        ff_accel_q8 <= ($signed({1'b0,move_speed}) - move_speed_d) * ff_gain_q8;
    end
end
wire signed [31:0] ff_term_q8 = ff_gain_q8==0 ? 32'sd0 : (ff_accel_q8 >>> 8);
wire signed [31:0] target_requested_q8 = (center_s <<< 8)-(pos_output_q8+vp_term_r+ff_term_q8);
wire signed [31:0] target_delta_q8 = target_requested_q8-angle_target_q8;
wire balanced_sample = tick && run_state == S_BAL && theta_in_range && !key_pend[3] && !mode_change;
wire pid_clear = (run_state != S_BAL) || (tick && key_pend[3]) || mode_change;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        d_stage1_q16<=0;d_stage2_q16<=0;d_phase<=0;
        d_error_hold<=0;d_int_limit_hold<=0;
        d_kp_hold<=0;d_ki_hold<=0;d_kd_hold<=0;d_out_limit_hold<=0;
    end else if(pid_clear || !theta_in_range) begin
        d_stage1_q16<=0;d_stage2_q16<=0;d_phase<=0;
    end else if(ANGLE_D_TWO_POLE && balanced_sample) begin
        d_stage1_q16<=d_first_next;d_phase<=1;
        d_error_hold<=angle_error_db_q8;d_int_limit_hold<=ANGLE_INT_LIM_ACT;
        d_kp_hold<=KP_A_ACT;d_ki_hold<=KI_A_ACT;d_kd_hold<=KD_A_ACT;d_out_limit_hold<=OUT_MAX_ACT;
    end else if(d_phase==1) begin d_stage2_q16<=d_second_next;d_phase<=2;end
    else if(d_phase==2) d_phase<=0;
end
// ---- 落地检测：goto_active 下降沿；静止死区：误差 < 3 计数 ----
reg goto_active_d;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) goto_active_d <= 1'b0;
    else       goto_active_d <= goto_active;
end
wire pos_landed = goto_active_d && !goto_active;      // 单周期脉冲
wire signed [31:0] pos_error_s32 = {{1{pos_error_db[30]}}, pos_error_db};
wire signed [31:0] pos_err_abs32 = pos_error_s32[31] ? -pos_error_s32 : pos_error_s32;
// 位置环积分门控：误差 <= 1°（≈3 计数）时冻结积分，避免在噪声底来回积分
wire pos_int_hold = (pos_err_abs32 <= 32'sd3);
wire pos_settle_db = !motion_active && (pos_err_abs32 < 32'sd768);   // 保留：1° 死区标志
wire signed [31:0] angle_value_q8, pos_value_q8;
wire angle_done, pos_done, angle_saturated, pos_saturated;
pid_fixed angle_pid(
    .clk_50m(clk_50m), .rst_n(rst_n), .clear(pid_clear),
    .start(ANGLE_D_TWO_POLE ? d_phase==2 : balanced_sample),
    .hold_integrator(1'b0), .error_q8(ANGLE_D_TWO_POLE ? d_error_hold : angle_error_db_q8), .derivative_q8(d_angle_next),
    .kp(ANGLE_D_TWO_POLE ? d_kp_hold : KP_A_ACT), .ki(ANGLE_D_TWO_POLE ? d_ki_hold : KI_A_ACT),
    .kd(ANGLE_D_TWO_POLE ? d_kd_hold : KD_A_ACT),
    .int_limit(ANGLE_D_TWO_POLE ? d_int_limit_hold : ANGLE_INT_LIM_ACT),
    .out_limit(ANGLE_D_TWO_POLE ? d_out_limit_hold : OUT_MAX_ACT),
    .value_q8(angle_value_q8), .done(angle_done), .saturated(angle_saturated),
    .integral_value_q8(trace_angle_integral_q8)
);
pid_fixed position_pid(
    .clk_50m(clk_50m), .rst_n(rst_n), .clear(pid_clear), .start(balanced_sample && pos_tick),
    .hold_integrator(pos_int_hold),
    .error_q8(pos_error_db <<< 8), .derivative_q8(d_pos_next),
    .kp(KP_P_EFF), .ki(KI_P_EFF), .kd(KD_P_ACT),
    .int_limit(POS_INT_LIM_ACT), .out_limit(POS_OUT_MAX_ACT),
    .value_q8(pos_value_q8), .done(pos_done), .saturated(pos_saturated),
    .integral_value_q8(trace_pos_integral_q8)
);
wire signed [31:0] duty_rounded = round_duty(duty_product);
wire signed [31:0] duty_directed = ANGLE_DIR < 0 ? -duty_rounded : duty_rounded;
wire signed [9:0] pid_command_sat = duty_directed > 255 ? 10'sd255 :
    duty_directed < -255 ? -10'sd255 : duty_directed[9:0];
// Friction compensation uses a smooth estimate of requested actuator effort,
// not the noisy sample's sign or the encoder's quantized velocity. This is an
// optional nonlinear actuator inverse, not trajectory velocity/acceleration FF.
// Only this bounded correction is smoothed; the PID drive path is unchanged.
reg signed [17:0] ff_effort_q8;
wire signed [19:0] ff_effort_weighted = (ff_effort_q8 <<< 1)+ff_effort_q8+
    (pid_command_sat <<< 8);
wire signed [19:0] ff_effort_next = ff_effort_weighted[19] ?
    -((-ff_effort_weighted) >>> 2) : (ff_effort_weighted >>> 2);
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) ff_effort_q8 <= 0;
    else if(pid_clear || !theta_in_range || friction_ff_ks==0 ||
        (usb_commit && (pending_friction_ff || pending_operation==3))) ff_effort_q8 <= 0;
    else if(duty_pending) ff_effort_q8 <= ff_effort_next[17:0];
end
wire [17:0] ff_effort_abs = ff_effort_q8[17] ? -ff_effort_q8 : ff_effort_q8;
// Soft transition over 16 PWM counts; correction reaches KS at |effort|>=16.
wire [8:0] ff_linear_q8 = ff_effort_abs >= 18'd4096 ? 9'd256 : {1'b0,ff_effort_abs[11:4]};
// C1 soft knees, no slope exceeds the old 1/16 linear transition:
// a<=4: a*a/128; 4<a<16: (a-2)/16;
// 16<=a<20: 1-(20-a)^2/128; a>=20: 1.
wire [17:0] ff_tail_q8 = 18'd5120-ff_effort_abs;
reg [6:0] ff_knee_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) ff_knee_r <= 7'd0;
    else       ff_knee_r <= ff_effort_abs <= 18'd1024 ?
        7'd31 : (ff_effort_abs >= 18'd5120 ? 7'd0 : ff_effort_abs[10:4]);
end
reg [13:0] ff_knee_square_r;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) ff_knee_square_r <= 14'd0;
    else       ff_knee_square_r <= ff_knee_r * ff_knee_r;
end
wire [13:0] ff_knee_square = ff_knee_square_r;
wire [6:0] ff_knee_q4_unused = ff_effort_abs <= 18'd1024 ?
    ff_effort_abs[10:4] : ff_effort_abs < 18'd5120 ? ff_tail_q8[10:4] : 7'd0;
wire [8:0] ff_curve_q8 = ff_effort_abs <= 18'd1024 ? {2'd0,ff_knee_square[13:7]} :
    ff_effort_abs < 18'd4096 ? ((ff_effort_abs-18'd512) >> 4) :
    ff_effort_abs < 18'd5120 ? 9'd256-{2'd0,ff_knee_square[13:7]} : 9'd256;
reg [8:0] ff_soft_intent_q8;
reg ff_soft_intent_sign;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin ff_soft_intent_q8<=0;ff_soft_intent_sign<=0;end
    else if(pid_clear || !theta_in_range || friction_ff_ks==0 ||
        (usb_commit && (pending_friction_ff || pending_operation==3))) begin
        ff_soft_intent_q8<=0;ff_soft_intent_sign<=0;
    end else begin ff_soft_intent_q8<=ff_curve_q8;ff_soft_intent_sign<=ff_effort_q8[17];end
end
wire [8:0] ff_intent_q8 = FRICTION_FF_SOFT_ENABLE ? ff_soft_intent_q8 : ff_linear_q8;
wire ff_intent_sign = FRICTION_FF_SOFT_ENABLE ? ff_soft_intent_sign : ff_effort_q8[17];
wire [8:0] ff_direction_scale = ff_intent_sign ? FRICTION_FF_NEG_SCALE_Q8 : FRICTION_FF_POS_SCALE_Q8;
wire [12:0] ff_scaled_product = friction_ff_ks*ff_direction_scale;
wire [13:0] ff_scaled_rounded = {1'b0,ff_scaled_product}+14'd128;
// Direction scales cannot raise the runtime KS ceiling.
wire [3:0] ff_effective_ks = ff_scaled_rounded[13:8] >= {2'd0,friction_ff_ks} ?
    friction_ff_ks : ff_scaled_rounded[11:8];
wire [12:0] ff_product_q8 = ff_effective_ks*ff_intent_q8;
wire [12:0] ff_rounded_q8 = ff_product_q8+13'd128;
// Register only the correction magnitude: keep the multiplier off the motor
// output/disable path. This adds20 ns to FF, none to the original PID path.
reg [4:0] ff_magnitude;
reg ff_magnitude_sign;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin ff_magnitude<=0;ff_magnitude_sign<=0;end
    else if(pid_clear || !theta_in_range || friction_ff_ks==0 ||
        (usb_commit && (pending_friction_ff || pending_operation==3))) begin
        ff_magnitude<=0;ff_magnitude_sign<=0;
    end else begin ff_magnitude<=ff_rounded_q8[12:8];ff_magnitude_sign<=ff_intent_sign;end
end
// Only FF growth is limited to 1 PWM per control result. Falling demand,
// zero, reversal and protection can withdraw correction immediately.
reg [4:0] ff_ramp_magnitude;
reg ff_ramp_sign;
wire [9:0] ff_pid_abs = pid_command_sat[9] ? -pid_command_sat : pid_command_sat;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin ff_ramp_magnitude<=0;ff_ramp_sign<=0;end
    else if(pid_clear || !theta_in_range || friction_ff_ks==0 ||
        (usb_commit && (pending_friction_ff || pending_operation==3))) begin
        ff_ramp_magnitude<=0;ff_ramp_sign<=0;
    end else if(duty_pending) begin
        ff_ramp_sign<=pid_command_sat[9];
        if(ff_pid_abs <= {1'b0,DEADZONE_ACT[8:0]} || ff_ramp_sign!=pid_command_sat[9] ||
           ff_effort_q8[17]!=pid_command_sat[9] || ff_magnitude_sign!=pid_command_sat[9])
            ff_ramp_magnitude<=0;
        else if(ff_magnitude < ff_ramp_magnitude) ff_ramp_magnitude<=ff_magnitude;
        else if(ff_magnitude > ff_ramp_magnitude) ff_ramp_magnitude<=ff_ramp_magnitude+5'd1;
    end
end
wire [9:0] ff_base_abs = u_duty[9] ? -u_duty : u_duty;
wire [9:0] ff_drive_room = ff_base_abs > {1'b0,DEADZONE_ACT[8:0]} ?
    ff_base_abs-{1'b0,DEADZONE_ACT[8:0]} : 10'd0;
wire [4:0] ff_soft_magnitude = ff_drive_room < {5'd0,ff_ramp_magnitude} ?
    ff_drive_room[4:0] : ff_ramp_magnitude;
wire [4:0] ff_applied_magnitude = FRICTION_FF_SOFT_ENABLE ? ff_soft_magnitude : ff_magnitude;
// Immediately remove a stale correction when PID changes direction or stops.
// The original PID deadzone still gates motor drive in the output block below.
// 摩擦前馈开关：运动过程中关闭；速度低于 18 计数/50ms 打开，
// 高于 30 才关闭（迟滞，避免临界抖动）
reg ff_slow_en;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) ff_slow_en <= 1'b1;
    else if(!motion_active) ff_slow_en <= 1'b1;
    else if(dpos_abs[23:8] > 16'd30) ff_slow_en <= 1'b0;
    else if(dpos_abs[23:8] < 16'd18) ff_slow_en <= 1'b1;
end

wire ff_apply = run_state==S_BAL && theta_in_range && !pid_clear && ff_slow_en &&
    friction_ff_ks!=0 && u_duty!=0 && ff_effort_q8!=0 &&
    (u_duty[9]==ff_effort_q8[17]) && (u_duty[9]==ff_magnitude_sign) &&
    (!FRICTION_FF_SOFT_ENABLE || u_duty[9]==ff_ramp_sign);
wire signed [10:0] ff_bias = !ff_apply ? 11'sd0 : u_duty[9] ?
    -$signed({6'd0,ff_applied_magnitude}) : $signed({6'd0,ff_applied_magnitude});
wire signed [10:0] compensated_duty = $signed({u_duty[9],u_duty})+ff_bias;
wire [10:0] compensated_magnitude = compensated_duty[10] ? -compensated_duty : compensated_duty;
wire [7:0] compensated_duty_u8 = compensated_magnitude>255 ? 8'd255 : compensated_magnitude[7:0];
wire signed [31:0] debug_rounded = round_q8(pos_value_q8);
// Optional historical experiment; disabled by default and in production top.
// Count only valid automatic balance samples. Restart the capture gain/timer
// after stop, retry or mode change; never alter the swing-up state machine.
// A UART KI write explicitly opts out of scheduling until reset, preserving
// the operator's gain command instead of silently replacing it later.
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        ki_a_auto_balance_r <= KI_A_AUTO;
        auto_capture_cnt <= 0; auto_ki_ramp_cnt <= 0; auto_ki_override <= 0;
    end else begin
        if(param_wr && !usb_pending && !manual_mode && param_type == 3'd1) auto_ki_override <= 1;
        if(usb_commit && !pending_bank && !pending_friction_ff) begin
            auto_ki_override <= pending_operation!=3;
            ki_a_auto_balance_r <= $signed(pending_gains[31:16]);
            auto_capture_cnt <= 0;auto_ki_ramp_cnt <= 0;
        end else if(run_state != S_BAL || manual_mode || mode_change || (tick && key_pend[3])) begin
            ki_a_auto_balance_r <= ki_a_auto_r;
            auto_capture_cnt <= 0; auto_ki_ramp_cnt <= 0;
        end else if(balanced_sample && auto_ki_scheduled) begin
            if(auto_capture_cnt < AUTO_KI_CAPTURE_TICKS) auto_capture_cnt <= auto_capture_cnt+1'b1;
            else if(ki_a_auto_balance_r > KI_A_AUTO_STEADY) begin
                if(auto_ki_ramp_cnt >= AUTO_KI_RAMP_DIV-1'b1) begin
                    auto_ki_ramp_cnt <= 0;
                    ki_a_auto_balance_r <= ki_a_auto_balance_r-1'b1;
                end else auto_ki_ramp_cnt <= auto_ki_ramp_cnt+1'b1;
            end
        end
    end
end
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        kp_a_man_r <= KP_A_MAN; ki_a_man_r <= KI_A_MAN; kd_a_man_r <= KD_A_MAN;
        kp_p_man_r <= KP_P_MAN; ki_p_man_r <= KI_P_MAN; kd_p_man_r <= KD_P_MAN;
        kp_a_auto_r <= KP_A_AUTO; ki_a_auto_r <= KI_A_AUTO; kd_a_auto_r <= KD_A_AUTO;
        kp_p_auto_r <= KP_P_AUTO; ki_p_auto_r <= KI_P_AUTO; kd_p_auto_r <= KD_P_AUTO;
        center_man_r <= ANGLE_CENTER_MAN; center_auto_r <= ANGLE_CENTER_AUTO;
        vp_gain_q8 <= 16'sd0;
        ff_gain_q8 <= 16'sd0;
    end else if(usb_commit && pending_operation==8'd13) begin
        // 平衡点中值（12bit）：bank 0=自动 1=手动
        if(pending_bank) center_man_r <= pending_gains[11:0];
        else center_auto_r <= pending_gains[11:0];
    end else if(usb_commit && pending_operation==8'd16) begin
        ff_gain_q8 <= pending_gains[15:0];
    end else if(usb_commit && pending_operation==8'd15) begin
        // 速度环 P 系数（Q8：256 = 100%，0 = 关闭）
        vp_gain_q8 <= pending_gains[15:0];
    end else if(usb_commit && !pending_position && !pending_friction_ff) begin
        if(pending_bank) begin
            {kd_p_man_r,ki_p_man_r,kp_p_man_r,kd_a_man_r,ki_a_man_r,kp_a_man_r} <= pending_gains;
        end else begin
            {kd_p_auto_r,ki_p_auto_r,kp_p_auto_r,kd_a_auto_r,ki_a_auto_r,kp_a_auto_r} <= pending_gains;
        end
    end else if(param_wr && !usb_pending) begin
        case(param_type)
            0: if(manual_mode) kp_a_man_r <= param_val; else kp_a_auto_r <= param_val;
            1: if(manual_mode) ki_a_man_r <= param_val; else ki_a_auto_r <= param_val;
            2: if(manual_mode) kd_a_man_r <= param_val; else kd_a_auto_r <= param_val;
            3: if(manual_mode) kp_p_man_r <= param_val; else kp_p_auto_r <= param_val;
            4: if(manual_mode) ki_p_man_r <= param_val; else ki_p_auto_r <= param_val;
            5: if(manual_mode) kd_p_man_r <= param_val; else kd_p_auto_r <= param_val;
            default: ;
        endcase
    end
end
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        run_state <= S_STOP; manual_mode <= MANUAL_START_DEF;
        judge_cnt <= 0; pos_cnt <= 0; count_time <= 0;
        zero_ref <= 0; pos_target <= 0; rot_goal <= 0; pos_slew_step <= ROT_STEP;
                                goto_active <= 1'b0; goto_target <= 16'sd0;
                                vmax_step <= ROT_STEP; move_dir <= 16'sd1;
                                move_speed <= 16'd1; rem_r <= 16'd0; goto_calc <= 1'b0;
        angle_target_q8 <= MANUAL_START_DEF ? (center_man_r << 8) : (center_auto_r << 8);
        pos_output_q8 <= 0; theta_prev <= 0; theta_d_prev <= 0; location_prev <= 0;
        d_angle_q8 <= 0; d_pos_q8 <= 0; ang_prev1 <= 0; ang_prev2 <= 0;
        fall_cnt <= 0; u_duty <= 0; duty_product <= 0; duty_pending <= 0; debug_out <= 0;
    end else begin
        duty_pending <= 0;
        // USB position commands are applied here so rot_goal/pos_slew_step keep
        // a single driver; the USB block only produces the acknowledge result.
        if(usb_commit && pending_position && run_state==S_BAL) begin
            if(pending_operation==8'd4) begin
                // Start a trapezoidal move towards the new target; op5 freezes.
                rot_goal <= pending_target_clamped;
                goto_target <= pending_target_clamped;
                vmax_step <= (pending_step<=16'sd0) ? ROT_STEP : pending_step;
                move_dir <= (cmd_delta < 0) ? -16'sd1 : 16'sd1;
                // 同方向续发目标时保留当前速度，否则每条命令都会从 1 档重新
                // 加速，看起来就是"起步一顿一顿、加不起速"。
                move_speed <= (goto_active && ((cmd_delta < 0) == move_dir[15])) ?
                     move_speed : 16'd1;
                rem_r <= cmd_mag[15:0];
                goto_calc <= 1'b0;
                goto_active <= (cmd_mag != 0);
            end else begin
                rot_goal <= pos_target;   // op5: freeze current position target
                goto_active <= 1'b0;
                rem_r <= 16'd0;
                goto_calc <= 1'b0;
            end
        end
        // Position profile phase 2: one cycle after a tick, re-compute the
        // remaining distance and the next speed from registered values only.
        if(goto_calc) begin
            rem_r <= rem_r-move_step;
            move_speed <= move_speed_next;
            goto_calc <= 1'b0;
        end
        if(angle_done && run_state == S_BAL && theta_in_range && !pid_clear) begin
            duty_product <= angle_value_q8*DUTY_SCALE;
            duty_pending <= 1;
        end
        if(ANGLE_D_TWO_POLE && d_phase==2 && run_state == S_BAL && theta_in_range && !pid_clear)
            d_angle_q8<=d_angle_next;
        if(duty_pending && run_state == S_BAL && theta_in_range && !pid_clear) begin
            if(duty_directed > 255) u_duty <= 10'sd255;
            else if(duty_directed < -255) u_duty <= -10'sd255;
            else u_duty <= duty_directed[9:0];
        end
        if(pos_done && run_state == S_BAL && !pid_clear) begin
            pos_output_q8 <= pos_value_q8+vp_term_r;
            debug_out <= debug_rounded[15:0];
        end
        if(tick) begin
            judge_cnt <= judge_tick ? 8'd0 : judge_cnt+1'b1;
            pos_cnt <= pos_tick ? 8'd0 : pos_cnt+1'b1;
            manual_mode <= requested_mode;
            // Start/stop has priority over every state action and PID result.
            if(key_pend[3]) begin
                u_duty <= 0; duty_pending <= 0; pos_output_q8 <= 0; debug_out <= 0;
                fall_cnt <= 0; judge_cnt <= 0; pos_cnt <= 0;
                if(run_state != S_STOP) run_state <= S_STOP;
                else if(requested_mode) begin
                    if(theta > requested_center-CENTER_RANGE_MAN && theta < requested_center+CENTER_RANGE_MAN) begin
                        run_state <= S_BAL; zero_ref <= arm_pos; pos_target <= 0; rot_goal <= 0;
                        pos_slew_step <= ROT_STEP;
                                goto_active <= 1'b0; goto_target <= 16'sd0;
                                vmax_step <= ROT_STEP; move_dir <= 16'sd1;
                                move_speed <= 16'd1; rem_r <= 16'd0; goto_calc <= 1'b0;
                        angle_target_q8 <= requested_center <<< 8;
                        theta_prev <= theta_s; theta_d_prev <= theta_s;
                        location_prev <= 0; d_angle_q8 <= 0; d_pos_q8 <= 0;
                    end
                end else run_state <= S_L1;
            end else if(mode_change && run_state == S_BAL) begin
                u_duty <= 0; duty_pending <= 0; pos_output_q8 <= 0; debug_out <= 0;
                angle_target_q8 <= requested_center <<< 8;
                theta_prev <= theta_s; theta_d_prev <= theta_s;
                location_prev <= location; d_angle_q8 <= 0; d_pos_q8 <= 0;
            end else begin
                if((key_pend[0] || key_pend[1] || key_pend[2]) && run_state == S_BAL) begin
                    rot_goal <= next_goal;
                    vmax_step <= 16'd16;
                    move_speed <= 16'd1;
                    move_dir <= next_goal[15] ? -16'sd1 : 16'sd1;
                    rem_r <= key_span_s[15] ? -key_span_s : key_span_s;
                    goto_calc <= 1'b0;
                end
                case(run_state)
                    S_STOP: u_duty <= 0;
                    S_JUDGE: begin
                        u_duty <= 0;
                        if(judge_tick) begin
                            if(theta > CENTER_HI && ang_prev1 > CENTER_HI && ang_prev2 > CENTER_HI &&
                               ang_prev1 < theta && ang_prev1 < ang_prev2) run_state <= S_L1;
                            else if(theta < CENTER_LO && ang_prev1 < CENTER_LO && ang_prev2 < CENTER_LO &&
                                    ang_prev1 > theta && ang_prev1 > ang_prev2) run_state <= S_R1;
                            else if(theta_in_range && ang_prev1 > CENTER_LO && ang_prev1 < CENTER_HI) begin
                                run_state <= S_BAL; zero_ref <= arm_pos; pos_target <= 0; rot_goal <= 0;
                        pos_slew_step <= ROT_STEP;
                                goto_active <= 1'b0; goto_target <= 16'sd0;
                                vmax_step <= ROT_STEP; move_dir <= 16'sd1;
                                move_speed <= 16'd1; rem_r <= 16'd0; goto_calc <= 1'b0;
                                pos_output_q8 <= 0; debug_out <= 0; pos_cnt <= 0; fall_cnt <= 0;
                                angle_target_q8 <= center_s <<< 8;
                                theta_prev <= theta_s; theta_d_prev <= theta_s;
                                location_prev <= 0; d_angle_q8 <= 0; d_pos_q8 <= 0;
                            end
                            ang_prev2 <= ang_prev1; ang_prev1 <= theta;
                        end
                    end
                    S_L1: begin u_duty <= SW_PWM; count_time <= SW_TIME; run_state <= S_L2; end
                    S_L2: if(count_time == 0) run_state <= S_L3; else count_time <= count_time-1'b1;
                    S_L3: begin u_duty <= -SW_PWM; count_time <= SW_TIME_RET; run_state <= S_L4; end
                    S_L4: if(count_time == 0) begin u_duty <= 0; run_state <= S_JUDGE; end
                          else count_time <= count_time-1'b1;
                    S_R1: begin u_duty <= -SW_PWM; count_time <= SW_TIME; run_state <= S_R2; end
                    S_R2: if(count_time == 0) run_state <= S_R3; else count_time <= count_time-1'b1;
                    S_R3: begin u_duty <= SW_PWM; count_time <= SW_TIME_RET; run_state <= S_R4; end
                    S_R4: if(count_time == 0) begin u_duty <= 0; run_state <= S_JUDGE; end
                          else count_time <= count_time-1'b1;
                    S_BAL: begin
                        theta_prev <= theta_s;
                        theta_d_prev <= theta_in_range ? theta_d_next : theta_s;
                        if(!theta_in_range) begin
                            // Never hold the last drive command while measurements are out of range.
                            u_duty <= 0; duty_pending <= 0; d_angle_q8 <= 0;
                            if(fall_cnt >= FALL_CNT_MAX) begin
                                fall_cnt <= 0;
                                if(manual_mode) run_state <= S_STOP;
                                else begin run_state <= S_L1; ang_prev1 <= 0; ang_prev2 <= 0; end
                            end else fall_cnt <= fall_cnt+1'b1;
                        end else begin
                            fall_cnt <= 0;
                            if(!ANGLE_D_TWO_POLE) d_angle_q8 <= d_angle_next;
                            if(target_delta_q8 > TARGET_STEP_Q8) angle_target_q8 <= angle_target_q8+TARGET_STEP_Q8;
                            else if(target_delta_q8 < -TARGET_STEP_Q8) angle_target_q8 <= angle_target_q8-TARGET_STEP_Q8;
                            else angle_target_q8 <= target_requested_q8;
                            if(pos_tick) begin
                                if(goto_active) begin
                                    if(lead_hold) begin
                                        // wait for the arm to catch up; also applies
                                        // to the final arrival step
                                    end else if(goto_arrive) begin
                                        // complete only once the arm is close and
                                        // slow; otherwise keep waiting at the
                                        // final approach position
                                        if(final_gate_ok) begin
                                            pos_target <= goto_target;
                                            goto_active <= 1'b0;
                                            goto_calc <= 1'b0;
                                        end
                                    end else begin
                                        pos_target <= pos_target_adv[15:0];
                                        goto_calc <= 1'b1;
                                    end
                                end else pos_target <= next_pos_target;
                                location_prev <= location; d_pos_q8 <= d_pos_next;
                            end
                        end
                    end
                    default: begin run_state <= S_STOP; u_duty <= 0; end
                endcase
            end
        end
    end
end
wire [9:0] u_mag = u_duty[9] ? -u_duty : u_duty;
assign trace_state = run_state;
assign trace_manual = manual_mode;
// Bits0..2: range/saturation; bit3: friction FF configured; bit4: motor active.
// Bit5: automatic KI reduced; bit6: reduced to steady KI.
// Bit7: active angle KI fits unsigned 8 bits; bits15..8: that actual gain.
wire auto_ki_reduced = auto_ki_scheduled && run_state == S_BAL && ki_a_auto_balance_r < ki_a_auto_r;
wire ki_trace_valid = KI_A_ACT >= 0 && KI_A_ACT <= 255;
assign trace_flags = {ki_trace_valid ? KI_A_ACT[7:0] : 8'd0,ki_trace_valid,
    (auto_ki_reduced && ki_a_auto_balance_r <= KI_A_AUTO_STEADY),auto_ki_reduced,
    (motor_sel != 0),(friction_ff_ks!=0),pos_saturated,angle_saturated,theta_in_range};
assign trace_location = location_mod;
assign trace_pos_target = pos_target;
assign trace_angle_target_q8 = angle_target_q8;
assign trace_pos_output_q8 = pos_output_q8;
assign trace_angle_error_q8 = angle_error_q8;
assign trace_d_angle_q8 = d_angle_q8;
assign trace_d_pos_q8 = d_pos_q8;
assign trace_u_duty = {{6{u_duty[9]}},u_duty};
// Read-only v2 configuration snapshot; low word is SW_PWM. Values reflect
// the selected mode and active runtime gains, not a local project manifest.
assign trace_config = {KD_P_ACT,KI_P_ACT,KP_P_ACT,KD_A_ACT,KI_A_ACT,KP_A_ACT,
    POS_DB_ACT,ANGLE_DB_ACT,{{14{ENC_DIR[1]}},ENC_DIR},
    4'b0,CENTER_RANGE_ACT,4'b0,ANGLE_CENTER_ACT,
    8'b0,SW_TIME_RET,8'b0,SW_TIME,{{7{SW_PWM[8]}},SW_PWM}};
always @* begin
    motor_sel = 2'b00; duty = 0;
    if(run_state != S_STOP) begin
        if(u_duty > DEADZONE_ACT) begin motor_sel = 2'b01; duty = compensated_duty_u8; end
        else if(u_duty < -DEADZONE_ACT) begin motor_sel = 2'b10; duty = compensated_duty_u8; end
    end
    if(run_state == S_BAL) state_out = 2'd2;
    else if(run_state == S_STOP) state_out = 2'd0;
    else state_out = 2'd1;
end
endmodule
