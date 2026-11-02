`timescale 1ns/1ns
// Convert a 16-bit unsigned value into five decimal digits in 16 clock cycles.
// Parallel constant / and % chains in the debug UART exceeded the 20 ns clock.
module bin_to_bcd(
    input clk_50m, input rst_n, input start, input [15:0] binary,
    output reg [19:0] bcd, output reg done
);
reg [15:0] shift_bin;
reg [19:0] work_bcd;
reg [3:0] bit_count;
reg busy;
reg [19:0] adjusted;
integer digit;
always @* begin
    adjusted=work_bcd;
    for(digit=0;digit<5;digit=digit+1)
        if(work_bcd[digit*4 +: 4] >= 5)
            adjusted[digit*4 +: 4]=work_bcd[digit*4 +: 4]+4'd3;
end
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        shift_bin <= 0; work_bcd <= 0; bit_count <= 0;
        busy <= 0; bcd <= 0; done <= 0;
    end else begin
        done <= 0;
        if(start) begin
            shift_bin <= binary; work_bcd <= 0; bit_count <= 0; busy <= 1;
        end else if(busy) begin
            work_bcd <= {adjusted[18:0],shift_bin[15]};
            shift_bin <= {shift_bin[14:0],1'b0};
            if(bit_count == 15) begin
                bcd <= {adjusted[18:0],shift_bin[15]}; done <= 1; busy <= 0;
            end else bit_count <= bit_count+1'b1;
        end
    end
end
endmodule
