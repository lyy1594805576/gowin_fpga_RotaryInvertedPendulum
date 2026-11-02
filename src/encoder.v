
`timescale 1ns/1ns
module encoder (

	input			clk_50m			,	// 50MHz时钟, 来自 top 的 clk_50m
    input			rst_n			,	// 复位(低有效), 来自 top 的 rst_n

    input			enc_a, enc_b	,	// 编码器A/B相(各1位方波), 来自 top 的 enc1_a/enc1_b(接XH2电机的编码器)

    output	[15:0]	pos					// 累计位置计数(4倍频, 上电初值10000), 送给 top 的 enc_pos1 → pendulum_ctrl.arm_pos

);

assign pos = pos_reg;

reg [1:0] enc_sync_a, enc_sync_b;
reg [1:0] state_now, state_last;

reg 	[15:0]	pos_reg;

reg 	[15:0]	pos_enc;
reg		[15:0]	pos_zero;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
		enc_sync_a <= 'd0;
		enc_sync_b <= 'd0;
		state_now <= 'd0;
	end else begin
		enc_sync_a <= {enc_sync_a[0], enc_a};
		enc_sync_b <= {enc_sync_b[0], enc_b};
		state_now <= {enc_sync_a[1], enc_sync_b[1]};
	end
end


// 四倍频方向判断，累加位置
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_reg <= 'd10000;
		pos_zero <= 0;
		pos_enc <= 'd0;
		//vel <= 0;
        state_last <= 2'b00;
    end else begin
        state_last <= state_now;
        case({state_last, state_now})

            4'b00_01: pos_reg <= pos_reg - 16'd1;
            4'b01_11: pos_reg <= pos_reg - 16'd1;
            4'b11_10: pos_reg <= pos_reg - 16'd1;
            4'b10_00: pos_reg <= pos_reg - 16'd1;

            4'b00_10: pos_reg <= pos_reg + 16'd1;
            4'b10_11: pos_reg <= pos_reg + 16'd1;
            4'b11_01: pos_reg <= pos_reg + 16'd1;
            4'b01_00: pos_reg <= pos_reg + 16'd1;
            default: pos_reg <= pos_reg;	//pos_reg
        endcase //vel <= pos_reg - pos;


    end
end



endmodule
