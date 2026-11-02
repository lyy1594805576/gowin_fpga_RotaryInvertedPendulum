`timescale 1ns/1ns
// USB 115200 8N1. 5A A5 01 23 seq16 op8 bank8 six signed32 Q12 CRC8 0D 0A.
// CRC8/ATM covers bytes 2..31. Bank 0=automatic, 1=manual.
// Ops 1=set gains, 2=read gains, 3=restore defaults (six signed32 Q12 gains).
// Op 4=position goto with a trapezoidal speed profile: payload word0 = signed
// target encoder counts relative to the balance-capture zero (|target|<=ARM_REV_MAX = 10 rev),
// word1 = signed profile peak speed in counts per 50 ms tick (0=firmware default,
// 1..64), words2..5 zero. The controller ramps 1 count/tick up to the peak,
// cruises and brakes symmetrically with no overshoot.
// Op 5=position hold: all payload words zero; freezes the position target at
// its current slewed value. Ops 4/5 are only accepted while balancing.
// Op7: friction feedforward, word0=-1 reads KS, or0..12 sets PWM compensation;
// words1..5 and bank must be0. Telemetry reply uses bank0x80 to distinguish
// KS readback from PID gains. Op6 remains the retired experiment (unsupported).
// Bad framing, incomplete packets, CRC, operation, bank and range never write
// gains or the position target.
module usb_pid_command #(
    parameter integer CLK_FREQ=50000000, BAUD=115200,
    parameter integer TIMEOUT_CLOCKS=5000000
)(
    input clk_50m, rst_n, rx,
    // High only in the S_BAL state: position commands are rejected otherwise.
    input balancing,
    output reg command_valid, error_valid,
    output reg [15:0] command_sequence,
    output reg [7:0] command_operation, command_bank, error_result,
    output reg [95:0] command_gains
);
localparam integer DIV=CLK_FREQ/BAUD;
// Result codes used by this module: 3 CRC, 4 parameter out of range,
// 5 unknown operation, 6 frame/timeout/framing, 8 position command while not
// balancing, 9 bank out of range. Result 2/7/1 come from the controller.
localparam signed [31:0] ARM_REV_MAX = 32'sd10630; // 10 rev x 1063 counts
reg [1:0] rx_sync, rx_state;
reg [15:0] rx_count;
reg [2:0] bit_index;
reg [7:0] rx_shift, byte_data;
reg byte_valid, framing_error;
always @(posedge clk_50m or negedge rst_n)
    if(!rst_n) rx_sync<=2'b11;
    else rx_sync<={rx_sync[0],rx};
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        rx_state<=0;rx_count<=0;bit_index<=0;rx_shift<=0;
        byte_data<=0;byte_valid<=0;framing_error<=0;
    end else begin
        byte_valid<=0;framing_error<=0;
        case(rx_state)
            0: if(!rx_sync[1]) begin rx_count<=0;rx_state<=1;end
            1: if(rx_count==DIV/2-1) begin
                rx_count<=0;
                if(!rx_sync[1]) begin rx_state<=2;bit_index<=0;end
                else rx_state<=0;
            end else rx_count<=rx_count+1'b1;
            2: if(rx_count==DIV-1) begin
                rx_count<=0;rx_shift[bit_index]<=rx_sync[1];
                if(bit_index==7) rx_state<=3;else bit_index<=bit_index+1'b1;
            end else rx_count<=rx_count+1'b1;
            3: if(rx_count==DIV-1) begin
                rx_count<=0;rx_state<=0;
                if(rx_sync[1]) begin byte_data<=rx_shift;byte_valid<=1;end
                else framing_error<=1;
            end else rx_count<=rx_count+1'b1;
        endcase
    end
end
reg [7:0] packet[0:34];
reg [5:0] index;
reg [22:0] wait_count;
reg [7:0] crc;
wire [15:0] received_sequence={packet[5],packet[4]};
wire [191:0] received_gains;
wire [5:0] range_ok;
wire signed [31:0] op4_target = received_gains[31:0];
wire signed [31:0] op4_step = received_gains[63:32];
wire op4_payload_ok = (received_gains[191:64]==128'd0) &&
    (op4_target>=-ARM_REV_MAX) && (op4_target<=ARM_REV_MAX) &&
    (op4_step>=32'sd0) && (op4_step<=32'sd64);
wire op5_payload_ok = (received_gains==96'd0);
wire op7_payload_ok = (received_gains[191:32]==160'd0) &&
    (op4_target>=-32'sd1) && (op4_target<=32'sd12);
genvar g;
generate for(g=0;g<6;g=g+1) begin: gains
    assign received_gains[g*32 +: 32]={packet[11+g*4],packet[10+g*4],packet[9+g*4],packet[8+g*4]};
    assign range_ok[g]=received_gains[g*32+16 +: 16]=={16{received_gains[g*32+15]}};
end endgenerate
function [7:0] crc_next;
    input [7:0] previous,value;
    reg [7:0] work;integer k;
    begin
        work=previous^value;
        for(k=0;k<8;k=k+1) work=work[7] ? (work<<1)^8'h07 : work<<1;
        crc_next=work;
    end
endfunction
integer j;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        index<=0;wait_count<=0;crc<=0;command_valid<=0;error_valid<=0;
        command_sequence<=0;command_operation<=0;command_bank<=0;error_result<=0;command_gains<=0;
        for(j=0;j<35;j=j+1) packet[j]<=0;
    end else begin
        command_valid<=0;error_valid<=0;
        if(index!=0) begin
            if(wait_count==TIMEOUT_CLOCKS-1) begin
                index<=0;wait_count<=0;error_valid<=1;error_result<=6;
                command_sequence<=index>=6 ? received_sequence : 16'd0;
                command_bank<=index>=8 ? packet[7] : 8'd0;
            end else wait_count<=wait_count+1'b1;
        end
        if(framing_error) begin index<=0;wait_count<=0;error_valid<=1;error_result<=6;
            command_sequence<=index>=6 ? received_sequence : 16'd0;
            command_bank<=index>=8 ? packet[7] : 8'd0;
        end
        if(byte_valid) begin
            wait_count<=0;packet[index]<=byte_data;
            case(index)
                0: if(byte_data==8'h5a) index<=1;
                1: if(byte_data==8'ha5) begin index<=2;crc<=0;end
                   else index<=byte_data==8'h5a ? 1 : 0;
                2: if(byte_data==1) begin index<=3;crc<=crc_next(0,byte_data);end
                   else index<=byte_data==8'h5a ? 1 : 0;
                3: if(byte_data==35) begin index<=4;crc<=crc_next(crc,byte_data);end
                   else index<=byte_data==8'h5a ? 1 : 0;
                32: index<=33;
                33: index<=34;
                34: begin
                    index<=byte_data==8'h5a ? 1 : 0;
                    command_sequence<=received_sequence;command_operation<=packet[6];command_bank<=packet[7];
                    if(packet[33]!=13 || byte_data!=10) begin error_valid<=1;error_result<=6;end
                    else if(packet[32]!=crc) begin error_valid<=1;error_result<=3;end
                    else if(packet[6]<1 || (packet[6]>5 && packet[6]!=7 && packet[6]!=13 && packet[6]!=15 && packet[6]!=16)) begin error_valid<=1;error_result<=5;end
                    else if(packet[7]>1 || (packet[6]==7 && packet[7]!=0)) begin error_valid<=1;error_result<=9;end
                    else if(packet[6]==1 && !(&range_ok)) begin error_valid<=1;error_result<=4;end
                    else if(packet[6]==4 && !op4_payload_ok) begin error_valid<=1;error_result<=4;end
                    else if(packet[6]==5 && !op5_payload_ok) begin error_valid<=1;error_result<=4;end
                    else if(packet[6]==7 && !op7_payload_ok) begin error_valid<=1;error_result<=4;end
                    else if(packet[6]==13 && (op4_target<0 || op4_target>4095))
                        begin error_valid<=1;error_result<=4;end
                    else if(packet[6]==15 && (op4_target<0 || op4_target>1024))
                        begin error_valid<=1;error_result<=4;end
                    else if((packet[6]==4 || packet[6]==5) && !balancing)
                        begin error_valid<=1;error_result<=8;end
                    else begin
                        command_valid<=1;
                        if(packet[6]==4)
                            command_gains<={48'd0,op4_step[15:0],op4_target};
                        else if(packet[6]==5)
                            command_gains<=96'd0;
                        else
                            for(j=0;j<6;j=j+1)
                                command_gains[j*16 +: 16]<=received_gains[j*32 +: 16];
                    end
                end
                default: begin index<=index+1'b1;crc<=crc_next(crc,byte_data);end
            endcase
        end
    end
end
endmodule
