`timescale 1ns/1ns
//===========================================================================
// dbg_uart.v — 调试串口: 把位置环输出、编码器位置、摆杆角度(含0.5s内最值)以
//             ASCII 十进制文本发送, 普通串口调试助手(文本模式)直接可读。
//---------------------------------------------------------------------------
// 帧格式(每 0.5s 一行, 固定 34 字节):
//   'P' 符号 万 千 百 十 个 ' ' 'A' 万 千 百 十 个 ' ' 'T' 千 百 十 个 ' ' 'L' 千 百 十 个 ' ' 'H' 千 百 十 个 '\r' '\n'
//   例:  P+00000 A10000 T3015 L2990 H3040\r\n
//   P = u_pos_t(位置环输出, 有符号, 实际限幅 ±300)
//   A = 编码器位置(无符号, 0~65535)
//   T = 摆杆角度 theta(当前值, 0~4095, 平衡点约 3015)
//   L = 本 0.5s 内 theta 最小值(锁存, 用于抓毛刺)
//   H = 本 0.5s 内 theta 最大值(锁存, 用于抓毛刺)
// 波特率 9600 8N1
//===========================================================================
module dbg_uart(
    input                  clk_50m,
    input                  rst_n,
    input  signed [15:0]   pos,     // 有符号数(位置环输出 u_pos_t 补码)
    input         [15:0]   arm,     // 无符号数(编码器位置)
    input         [11:0]   theta,   // 摆杆角度(ADC, 0~4095)
    output                 tx
);

localparam [24:0] PERIOD = 25'd24_999_999;   // 0.5s

// ---- 0.5s 定时触发 ----
reg [24:0] cnt;
reg        trig;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        cnt  <= 25'd0;
        trig <= 1'b0;
    end else begin
        trig <= 1'b0;
        if(cnt == PERIOD) begin
            cnt  <= 25'd0;
            trig <= 1'b1;
        end else
            cnt <= cnt + 1'b1;
    end
end

// ---- 锁存本次要发送的数值(保证一整帧内数值一致) ----
reg signed [15:0] pos_l;
reg        [15:0] arm_l;
reg        [11:0] theta_l;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_l   <= 16'sd0;
        arm_l   <= 16'd0;
        theta_l <= 12'd0;
    end else if(trig) begin
        pos_l   <= pos;
        arm_l   <= arm;
        theta_l <= theta;
    end
end

// ---- 0.5s 内 theta 最小/最大值锁存(每个时钟都跟踪, 抓 3ms 级毛刺) ----
reg [11:0] theta_min, theta_max;   // 当前 0.5s 窗口内累计的最值
reg [11:0] theta_min_l, theta_max_l; // 锁存到帧里的最值
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        theta_min   <= 12'hFFF;
        theta_max   <= 12'd0;
        theta_min_l <= 12'd0;
        theta_max_l <= 12'd0;
    end else begin
        if(theta < theta_min) theta_min <= theta;
        if(theta > theta_max) theta_max <= theta;
        if(trig) begin
            theta_min_l <= theta_min;
            theta_max_l <= theta_max;
            theta_min   <= 12'hFFF;
            theta_max   <= 12'd0;
        end
    end
end

// ---- 16-clock binary-to-BCD conversion, completed before the second UART byte ----
wire [15:0] abs_pos = pos_l[15] ? (-pos_l) : pos_l;
reg convert_start;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) convert_start <= 0;
    else convert_start <= trig; // snapshot registers are now updated
end
wire [19:0] p_bcd;
bin_to_bcd p_convert(.clk_50m(clk_50m),.rst_n(rst_n),.start(convert_start),.binary(abs_pos),.bcd(p_bcd),.done());
wire [3:0] p0 = p_bcd[19:16];
wire [3:0] p1 = p_bcd[15:12];
wire [3:0] p2 = p_bcd[11:8];
wire [3:0] p3 = p_bcd[7:4];
wire [3:0] p4 = p_bcd[3:0];
wire [19:0] a_bcd;
bin_to_bcd a_convert(.clk_50m(clk_50m),.rst_n(rst_n),.start(convert_start),.binary(arm_l),.bcd(a_bcd),.done());
wire [3:0] a0 = a_bcd[19:16];
wire [3:0] a1 = a_bcd[15:12];
wire [3:0] a2 = a_bcd[11:8];
wire [3:0] a3 = a_bcd[7:4];
wire [3:0] a4 = a_bcd[3:0];
wire [19:0] t_bcd;
bin_to_bcd t_convert(.clk_50m(clk_50m),.rst_n(rst_n),.start(convert_start),.binary({4'b0,theta_l}),.bcd(t_bcd),.done());
wire [3:0] t0 = t_bcd[15:12];
wire [3:0] t1 = t_bcd[11:8];
wire [3:0] t2 = t_bcd[7:4];
wire [3:0] t3 = t_bcd[3:0];
wire [19:0] l_bcd;
bin_to_bcd l_convert(.clk_50m(clk_50m),.rst_n(rst_n),.start(convert_start),.binary({4'b0,theta_min_l}),.bcd(l_bcd),.done());
wire [3:0] l0 = l_bcd[15:12];
wire [3:0] l1 = l_bcd[11:8];
wire [3:0] l2 = l_bcd[7:4];
wire [3:0] l3 = l_bcd[3:0];
wire [19:0] h_bcd;
bin_to_bcd h_convert(.clk_50m(clk_50m),.rst_n(rst_n),.start(convert_start),.binary({4'b0,theta_max_l}),.bcd(h_bcd),.done());
wire [3:0] h0 = h_bcd[15:12];
wire [3:0] h1 = h_bcd[11:8];
wire [3:0] h2 = h_bcd[7:4];
wire [3:0] h3 = h_bcd[3:0];

wire [7:0] ch_sign = pos_l[15] ? 8'h2D : 8'h2B;    // '-' / '+'

// 根据字节序号取帧内字节
function [7:0] get_byte;
    input [5:0] i;
    begin
        case(i)
            0:  get_byte = 8'h50;                 // 'P'
            1:  get_byte = ch_sign;
            2:  get_byte = p0[3:0] + 8'h30;
            3:  get_byte = p1[3:0] + 8'h30;
            4:  get_byte = p2[3:0] + 8'h30;
            5:  get_byte = p3[3:0] + 8'h30;
            6:  get_byte = p4[3:0] + 8'h30;
            7:  get_byte = 8'h20;                 // ' '
            8:  get_byte = 8'h41;                 // 'A'
            9:  get_byte = a0[3:0] + 8'h30;
            10: get_byte = a1[3:0] + 8'h30;
            11: get_byte = a2[3:0] + 8'h30;
            12: get_byte = a3[3:0] + 8'h30;
            13: get_byte = a4[3:0] + 8'h30;
            14: get_byte = 8'h20;                 // ' '
            15: get_byte = 8'h54;                 // 'T'
            16: get_byte = t0[3:0] + 8'h30;
            17: get_byte = t1[3:0] + 8'h30;
            18: get_byte = t2[3:0] + 8'h30;
            19: get_byte = t3[3:0] + 8'h30;
            20: get_byte = 8'h20;                 // ' '
            21: get_byte = 8'h4C;                 // 'L'
            22: get_byte = l0[3:0] + 8'h30;
            23: get_byte = l1[3:0] + 8'h30;
            24: get_byte = l2[3:0] + 8'h30;
            25: get_byte = l3[3:0] + 8'h30;
            26: get_byte = 8'h20;                 // ' '
            27: get_byte = 8'h48;                 // 'H'
            28: get_byte = h0[3:0] + 8'h30;
            29: get_byte = h1[3:0] + 8'h30;
            30: get_byte = h2[3:0] + 8'h30;
            31: get_byte = h3[3:0] + 8'h30;
            32: get_byte = 8'h0D;                 // '\r'
            33: get_byte = 8'h0A;                 // '\n'
            default: get_byte = 8'h0A;
        endcase
    end
endfunction

// ---- 字节发送状态机 ----
localparam [1:0] IDLE = 2'd0, SEND = 2'd1, WAIT = 2'd2;
reg [1:0]  state;
reg [5:0]  byte_idx;   // 0~33, 需要 6bit
reg        flag_r;     // 触发 uart_tx 发送一字节(单周期脉冲)
reg [7:0]  byte_r;
wire       pi_done;
reg [15:0] gap_cnt;    // 字节间间隔计数(避免背靠背连发导致部分串口芯片失步)
localparam [15:0] GAP_CNT = 16'd50000;  // 1ms 间隔(50MHz)

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        state    <= IDLE;
        byte_idx <= 6'd0;
        flag_r   <= 1'b0;
        byte_r   <= 8'd0;
        gap_cnt  <= 16'd0;
    end else begin
        flag_r <= 1'b0;
        case(state)
            IDLE: begin
                if(trig) begin
                    byte_idx <= 6'd0;
                    byte_r   <= get_byte(6'd0);
                    flag_r   <= 1'b1;
                    state    <= SEND;
                end
            end
            SEND: begin
                if(pi_done) begin
                    if(byte_idx == 6'd33) begin
                        state <= IDLE;
                    end else begin
                        gap_cnt <= GAP_CNT;
                        state   <= WAIT;
                    end
                end
            end
            WAIT: begin
                if(gap_cnt == 16'd0) begin
                    byte_idx <= byte_idx + 1'b1;
                    byte_r   <= get_byte(byte_idx + 1'b1);
                    flag_r   <= 1'b1;
                    state    <= SEND;
                end else
                    gap_cnt <= gap_cnt - 1'b1;
            end
            default: state <= IDLE;
        endcase
    end
end

// uart_tx 单字节发送
uart_tx #(
    .UART_BPS(21'd9600),
    .CLK_FREQ(26'd50_000_000)
) uart_tx_inst(
    .sys_clk   (clk_50m),
    .sys_rst_n (rst_n),
    .pi_data   (byte_r),
    .pi_flag   (flag_r),
    .tx        (tx),
    .pi_done   (pi_done)
);

endmodule
