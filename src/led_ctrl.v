`timescale 1ns/1ns
module led_ctrl(

    input			clk_50m 	,	// 50MHz时钟, 来自 top 的 clk_50m
    input			rst_n   	,	// 复位(低有效), 来自 top 的 rst_n

    input	[3:0]	key_flag	,	// 4路按键消抖脉冲(按下一次=1拍高电平), 来自 top 的 swing_key[3:0]
                                    // key_flag[0]=SW1  key_flag[1]=SW2  key_flag[2]=SW3  key_flag[3]=SW4

    input	[1:0]	ctrl_state	,	// 控制状态, 来自 top 的 ctrl_state(pendulum_ctrl 的 state_out)
                                    // 2'd0=空闲(IDLE)  2'd1=起摆中(SWING)  2'd2=平衡(BALANCE)

    output	reg	[3:0]	led			// 4路LED(低电平点亮), 送给 top 的 led[3:0]
                                    // led[1:0] = 状态显示: 全灭=IDLE  led0亮=起摆  led1亮=平衡
                                    // led[2] = SW3 按下翻转  led[3] = SW4 按下翻转(按键演示)

    );

// 低电平点亮: bit=0 灯亮, bit=1 灯灭
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        led <= 4'b1111;                // 复位: 全灭
    else
        led <= { led[3] ^ key_flag[3], // SW4 按下翻转(演示按键是否被检测到)
                 led[2] ^ key_flag[2], // SW3 按下翻转
                 ~ctrl_state[1],       // 状态高bit: BALANCE 时 =0 → led1 亮
                 ~ctrl_state[0] };     // 状态低bit: SWING   时 =0 → led0 亮
end

endmodule
