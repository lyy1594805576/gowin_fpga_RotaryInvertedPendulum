`timescale 1ns/1ns
// Decode the physical UART output, including the full stop bit, at 115200 baud.
module trace_uart_tb;
reg clk=0;always #10 clk=~clk;
reg rst_n=0;
reg [5:0] state=21;
reg [11:0] theta=3017;
wire tx;
pendulum_trace_uart dut(.clk_50m(clk),.rst_n(rst_n),.state(state),.manual_mode(1'b0),
    .flags(16'd3217),.arm(16'hffff),.location(-16'sd3),.pos_target(16'sd260),
    .theta_raw(12'd3018),.theta(theta),.angle_target_q8(32'sd772352),
    .pos_output_q8(-32'sd76800),.angle_error_q8(-32'sd256),
    .d_angle_q8(-32'sd128),.d_pos_q8(32'sd512),.u_duty(-16'sd180),
    .requested_pwm(-16'sd180),.angle_integral_q8(-32'sd25600),
    .pos_integral_q8(32'sd1280000),
    .config_words({16'sd9950,16'sd16,16'sd940,16'sd31600,16'sd12,16'sd1550,
        16'sd2,16'sd0,-16'sd1,16'sd500,16'sd3017,16'sd95,16'sd65,16'sd180}),
    .pid_command_sequence(16'd17),.pid_active_sequence(16'd16),.pid_command_result(8'd7),
    .pid_command_bank(8'd1),
    .pid_readback_gains({16'sd8000,16'sd4,16'sd870,16'sd16500,16'sd4,16'sd1750}),.tx(tx));
task receive_byte;
output [7:0] byte_value;
integer b;
begin
    @(negedge tx);
    repeat(651)@(posedge clk);#1;
    for(b=0;b<8;b=b+1)begin byte_value[b]=tx;repeat(434)@(posedge clk);#1;end
    if(tx!==1'b1)$fatal(1,"UART stop bit missing");
end
endtask
integer frame,i,fd;
reg [7:0] value;
time start1,start2;
initial begin
    fd=$fopen("sim/trace_uart_fixture.hex","w");
    #25;rst_n=1;
    @(negedge clk);dut.uptime_ms=123456;dut.ms_cnt=0;dut.packet_seq=65535;
    dut.theta_min=3000;dut.theta_max=3030;dut.period_cnt=499999;
    for(frame=0;frame<2;frame=frame+1)begin
        for(i=0;i<111;i=i+1)begin
            receive_byte(value);$fwrite(fd,"%02x\n",value);
            if(i==0)begin if(frame==0)start1=$time;else start2=$time;end
            if(frame==0 && i==5)begin state=4;theta=3100;end
        end
    end
    if(start2-start1!=10000000)$fatal(1,"telemetry frame interval is not 10ms");
    $fclose(fd);
    $display("PASS 115200 physical UART, complete stop bits and 100Hz frame cadence");
    $display("PASS trace_uart_tb");$finish;
end
initial begin #21000000;$fatal(1,"trace UART timeout");end
endmodule
