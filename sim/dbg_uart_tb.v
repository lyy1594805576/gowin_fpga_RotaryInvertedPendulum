`timescale 1ns/1ns
module dbg_uart_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0;
reg signed [15:0] pos=-300;
reg [15:0] arm=65535;
reg [11:0] theta=3017;
dbg_uart dut(.clk_50m(clk),.rst_n(rst_n),.pos(pos),.arm(arm),.theta(theta),.tx());
integer i;
reg [7:0] expected [0:33];
initial begin
    {expected[0],expected[1],expected[2],expected[3],expected[4],expected[5],expected[6],
     expected[7],expected[8],expected[9],expected[10],expected[11],expected[12],expected[13],
     expected[14],expected[15],expected[16],expected[17],expected[18],expected[19],
     expected[20],expected[21],expected[22],expected[23],expected[24],expected[25],
     expected[26],expected[27],expected[28],expected[29],expected[30],expected[31],
     expected[32],expected[33]}="P-00300 A65535 T3017 L3000 H3030\r\n";
    expected[32]=8'h0d; expected[33]=8'h0a;
    #25; rst_n=1;
    @(negedge clk); dut.theta_min=3000; dut.theta_max=3030; dut.cnt=25'd24999999;
    wait(dut.trig);
    repeat(24) @(posedge clk); #1;
    for(i=0;i<34;i=i+1) if(dut.get_byte(i) !== expected[i])
        $fatal(1,"UART byte %0d: got %h expected %h",i,dut.get_byte(i),expected[i]);
    $display("PASS debug UART frame: P-00300 A65535 T3017 L3000 H3030");
    $display("PASS dbg_uart_tb"); $finish;
end
initial begin #10000; $fatal(1,"debug UART test timeout"); end
endmodule
