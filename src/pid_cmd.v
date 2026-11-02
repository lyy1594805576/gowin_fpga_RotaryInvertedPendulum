`timescale 1ns/1ns
//===========================================================================
// pid_cmd.v — 蓝牙串口命令解析器
//   从 UART 接收 ASCII 文本命令, 解析成 PID 参数写入总线
//   (param_type / param_val / param_wr), 交给 pendulum_ctrl 改写内外环 PID。
//---------------------------------------------------------------------------
// 命令(不区分大小写, 以回车 '\r' 或换行 '\n' 结尾):
//   KPA <值>   角度环 KP (类型 0)     KPP <值>   位置环 KP (类型 3)
//   KIA <值>   角度环 KI (类型 1)     KIP <值>   位置环 KI (类型 4)
//   KDA <值>   角度环 KD (类型 2)     KDP <值>   位置环 KD (类型 5)
//   例: 手机发 "KPA 1300" 回车 -> param_type=0, param_val=1300, param_wr 一个脉冲
// 波特率 9600 8N1 (与 dbg_uart 一致)
//===========================================================================
module pid_cmd(
    input             clk_50m,    // 50MHz
    input             rst_n,      // 复位(低有效)
    input             rx,         // 串口接收(HC-04 的 TXD -> FPGA uart_rx)
    output reg [2:0]  param_type, // 参数类型 0~5
    output reg [15:0] param_val,  // 参数值(有符号)
    output reg        param_wr    // 写脉冲(1 个时钟高电平)
);

// ---- 串口接收 8N1 9600 ----
wire [7:0] rx_data;
wire       rx_flag;
uart_rx #(
    .UART_BPS (16'd9600),
    .CLK_FREQ (26'd50_000_000)
) u_rx (
    .sys_clk   (clk_50m),
    .sys_rst_n (rst_n),
    .rx        (rx),
    .po_data   (rx_data),
    .po_flag   (rx_flag)
);

// ---- 接收字节锁存(po_flag 是单周期脉冲) ----
reg [7:0] ch;
reg       ch_v;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        ch   <= 8'd0;
        ch_v <= 1'b0;
    end else if(rx_flag) begin
        ch   <= rx_data;
        ch_v <= 1'b1;
    end else begin
        ch_v <= 1'b0;
    end
end

// ---- 转大写 + 字符类别判断 ----
wire [7:0] ch_u     = (ch >= 8'h61 && ch <= 8'h7A) ? (ch - 8'h20) : ch;
wire       is_alpha = (ch_u >= 8'h41 && ch_u <= 8'h5A);
wire       is_digit = (ch_u >= 8'h30 && ch_u <= 8'h39);

// ---- 解析状态机: IDLE -> CMD -> NUM -> (换行) 输出写脉冲 ----
localparam [1:0] S_IDLE = 2'd0, S_CMD = 2'd1, S_NUM = 2'd2;
reg [1:0]         state;
reg [23:0]        cmd;     // 最近 3 个命令字符(低位为最新)
reg signed [31:0] num;     // 数字累加(32bit 防溢出)
reg               neg;     // 负号
reg [3:0]         num_i;   // 已收数字位数(0~5)

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        state      <= S_IDLE;
        cmd        <= 24'd0;
        num        <= 32'sd0;
        neg        <= 1'b0;
        num_i      <= 4'd0;
        param_type <= 3'd0;
        param_val  <= 16'sd0;
        param_wr   <= 1'b0;
    end else begin
        param_wr <= 1'b0;
        if(ch_v) begin
            case(state)
                S_IDLE: begin
                    cmd   <= 24'd0;
                    num   <= 32'sd0;
                    neg   <= 1'b0;
                    num_i <= 4'd0;
                    if(is_alpha) begin
                        cmd   <= {16'd0, ch_u};
                        state <= S_CMD;
                    end
                end
                S_CMD: begin
                    if(is_alpha) begin
                        cmd <= {cmd[15:0], ch_u};
                    end else if(ch == 8'h20) begin
                        state <= S_NUM;
                    end else if(ch_u == 8'h2D) begin   // '-'
                        neg   <= 1'b1;
                        state <= S_NUM;
                    end else if(is_digit) begin
                        num   <= {28'd0, ch_u[3:0]};
                        num_i <= 4'd1;
                        state <= S_NUM;
                    end else if(ch == 8'h0D || ch == 8'h0A) begin
                        state <= S_IDLE;   // 只有命令没有数字, 忽略
                    end
                end
                S_NUM: begin
                    if(is_digit) begin
                        if(num_i < 4'd5) begin
                            num   <= num * 32'sd10 + $signed({28'd0, ch_u[3:0]});
                            num_i <= num_i + 4'd1;
                        end
                    end else if(ch == 8'h0D || ch == 8'h0A) begin
                        if(cmd_type(cmd) != 3'd7) begin   // 命令合法才写
                            param_type <= cmd_type(cmd);
                            param_val  <= neg ? (-$signed(num[15:0])) : num[15:0];
                            param_wr   <= 1'b1;
                        end
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
end

// ---- 命令(3 字符) -> 参数类型 ----
function [2:0] cmd_type;
    input [23:0] c;
    begin
        case(c)
            24'h4B5041: cmd_type = 3'd0;   // "KPA"
            24'h4B4941: cmd_type = 3'd1;   // "KIA"
            24'h4B4441: cmd_type = 3'd2;   // "KDA"
            24'h4B5050: cmd_type = 3'd3;   // "KPP"
            24'h4B4950: cmd_type = 3'd4;   // "KIP"
            24'h4B4450: cmd_type = 3'd5;   // "KDP"
            default:    cmd_type = 3'd7;   // 非法
        endcase
    end
endfunction

endmodule
