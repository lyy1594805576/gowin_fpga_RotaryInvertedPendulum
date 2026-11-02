create_clock -name clk_50m -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50m}]
# Asynchronous inputs terminate in existing synchronizers/debounce circuits.
set_false_path -from [get_ports {rst_n key_sw[0] key_sw[1] key_sw[2] key_sw[3] enc1_a enc1_b enc2_a enc2_b uart_rx hc04_rx}]
# ADC board delays and output clock/data phase require hardware measurements;
# do not invent input delays or exempt any internal PID arithmetic from timing.
