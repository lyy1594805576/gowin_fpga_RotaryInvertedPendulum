`timescale 1ns/1ns
module bin_to_bcd_tb;
reg clk=0; always #10 clk=~clk;
reg rst_n=0,start=0;
reg [15:0] binary=0;
wire [19:0] bcd;
wire done;
bin_to_bcd dut(.clk_50m(clk),.rst_n(rst_n),.start(start),.binary(binary),.bcd(bcd),.done(done));
task check;
input [15:0] x;
integer expected,actual,i,cycles;
begin
    @(negedge clk); binary=x; start=1;
    @(negedge clk); start=0; cycles=0;
    while(!done && cycles<20) begin @(negedge clk); cycles=cycles+1; end
    if(cycles!=16) $fatal(1,"BCD completion latency incorrect");
    actual=0;
    for(i=4;i>=0;i=i-1) begin
        if(bcd[i*4 +: 4]>9) $fatal(1,"invalid decimal digit");
        actual=actual*10+bcd[i*4 +: 4];
    end
    if(actual!=x) $fatal(1,"BCD conversion incorrect: input=%0d got=%0d",x,actual);
end
endtask
integer n;
initial begin
    #25; rst_n=1;
    check(0); check(1); check(9); check(10); check(9999); check(10000);
    check(3017); check(32768); check(65535);
    for(n=0;n<100;n=n+1) check($random);
    $display("PASS binary-to-decimal boundaries and 100 varied inputs");
    $display("PASS bin_to_bcd_tb"); $finish;
end
initial begin #100000; $fatal(1,"BCD test timeout"); end
endmodule
