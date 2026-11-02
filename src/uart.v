`timescale 1ns/1ns
module uart #(

    parameter   CNT_1S = 32'd99_999_999 //写入后延时1S读取
	
) (

	input              		clk_50M		,	// 50MHz时钟, 来自 top 的 clk_50m
	input              		rst_n		,	// 复位(低有效), 来自 top 的 rst_n

	input					key3		,	// SW3 按键脉冲(按下触发一帧发送), 来自 top 的 swing_key[2]

	input              		fpga_rx		,	// 串口接收, 来自 top 的 uart_rx(本工程未使用, 未例化uart_rx)
	output             		fpga_tx		,	// 串口发送, 送给 top 的 uart_tx(接USB转串口)

	input      		[15:0]	data_in		,	// 摆杆角度, 来自 top 的 {4'b0, theta_out}
	input			[31:0]	encoder_in	,	// 转臂位置, 来自 top 的 {enc_pos1, enc_pos1}

	output	reg		[15:0]	data_out      // 串口接收数据(本工程未使用, 恒为0)
	
);

parameter	UART_BPS       =   21'd9600			,   //比特率
            CLK_FREQ       =   26'd50_000_000	,   //时钟频率            
            SEN_CNT_MAX    =   20'd999_999		,   //50M 计数1.5ms 发一个byte数据 > 1.04ms 9600bps
            SEN_BYTE_CNT   =   4'd9				;	//发送字节数量 

wire			pi_flag		;
wire	[7:0]	pi_data		;
wire			pi_done		;



uart_tx #(

    .UART_BPS    (UART_BPS  ),  //串口波特率
    .CLK_FREQ    (CLK_FREQ  )   //时钟频率
	
) uart_tx_inst (

    .sys_clk    (clk_50M    ),  
    .sys_rst_n  (rst_n      ),  
    .pi_data    (pi_data    ),  
    .pi_flag    (pi_flag    ),  
                
    .tx         (fpga_tx    ),  
    .pi_done    (pi_done    )
	
);

reg [21:0]  send_cnt;
reg	[3:0]	rx_byte_cnt	;
reg	[7:0]	cmdx		;
reg	[7:0]	data0		;
reg	[7:0]	data1		;

reg	u_wr_r	;
reg	u_wr_rr	;

reg	u_wr_pos	;
reg	u_wr_neg	;

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_wr_r   <= 1'b0;
		u_wr_rr  <= 1'b0;
		u_wr_neg <= 1'b0;
	end else begin
		u_wr_r   <= key3;
		u_wr_rr  <= u_wr_r;
		u_wr_pos <= u_wr_r & ~u_wr_rr;
		u_wr_neg <= ~u_wr_r & u_wr_rr;
	end
end

reg			u_wr_valid	;
reg			u_rd_pos	;
reg	[31:0]	cnt_delay	;

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_wr_valid    <= 1'b0;
	end else begin
		if(u_wr_pos) 
			u_wr_valid = 1'b1;
		if(cnt_delay == CNT_1S)begin 
			u_wr_valid = 1'b0;
		end
	end
end

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		cnt_delay   <= 'd0;
	end else begin
		if(u_wr_valid)begin
			if (cnt_delay == CNT_1S) begin   
				cnt_delay <= 'd0;
			end else begin
				cnt_delay <= cnt_delay + 'd1;
			end
		end
	end
end

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_rd_pos    <= 1'b0;
	end else begin
		if (cnt_delay == CNT_1S)
			u_rd_pos <= 1'b1;
		else 
			u_rd_pos <= 1'b0;
	end
end

//------------------------Print message via serial port---------------------------------------//
reg [4:0]   tx_byte_cnt;
wire        send_en;
reg [7:0]   pi_data_r;

assign pi_flag = send_en;
assign pi_data = pi_data_r;

reg send_valid;

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n==1'b0)
        send_valid <= 1'b0;
    else if(u_rd_pos)		
        send_valid <= 1'b1;
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        send_cnt <= 'd0;
    end else begin
        if(send_valid && send_cnt < SEN_CNT_MAX && tx_byte_cnt <= SEN_BYTE_CNT)	
            send_cnt <= send_cnt + 1'b1;
        else 
            send_cnt <= 'd0;
    end
end
 
assign send_en = (send_cnt == SEN_CNT_MAX - 1) ? 1'b1 : 1'b0;

reg		[31:0]	count_s;


always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        count_s <= 'd0;
	end else if(count_s == CNT_1S - 'd1) begin
		count_s <= 'd0;
	end else begin
		count_s <= count_s + 'd1;
	end
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        tx_byte_cnt <= 5'd0;
    end else begin		
		//if(u_rd_pos || tx_byte_cnt >= 'd6)	//解除发送字节限制		
		if( u_rd_pos || (count_s == CNT_1S - 'd1) )	//解除发送字节限制	
            tx_byte_cnt <= 5'd0; 
        else if(tx_byte_cnt > SEN_BYTE_CNT)
            tx_byte_cnt <= SEN_BYTE_CNT + 1; 
        else if(send_en)
            tx_byte_cnt <= tx_byte_cnt +1'b1; 
    end
end

reg	[15:0]	data_in_reg		;
reg	[31:0]	encoder_in_reg	;

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		data_in_reg <= 'd0;
		encoder_in_reg <= 'H4E20;
	end else if(send_en) begin
		data_in_reg <= data_in;
		encoder_in_reg <= encoder_in;		
	end else begin
		data_in_reg <= data_in_reg;	
		encoder_in_reg <= encoder_in_reg;	
	end
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        pi_data_r <= 'd0;
	end else if( send_en )begin
		case(tx_byte_cnt)
			0:	begin 
					pi_data_r <= 'HAA; 
				end
			1:	begin 
					pi_data_r <= 'H55; 
				end
			2:	begin 
					pi_data_r <= data_in_reg[15:8]; 
				end
			3:	begin 
					pi_data_r <= data_in_reg[7:0]; 
				end
			4:	begin 
					pi_data_r <= encoder_in_reg[31:24]; 
				end
			5:	begin 
					pi_data_r <= encoder_in_reg[23:16]; 
				end		
			6:	begin 
					pi_data_r <= encoder_in_reg[15:8]; 
				end
			7:	begin 
					pi_data_r <= encoder_in_reg[7:0];
				end
			8:	begin 
					pi_data_r <= 'H0D; 
				end
			9:	begin 
					pi_data_r <= 'H0A;  
				end
		endcase
			

	end
end

endmodule