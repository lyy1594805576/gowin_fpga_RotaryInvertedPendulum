set project_dir [file dirname [file normalize [info script]]]
open_project "$project_dir/jiangxiedaolibai.gprj"
set_option -top_module top
# key_sw[3] sits on a dual-purpose SSPI pin; fresh impl/ folders default SSPI
# to a dedicated pin and pnr then refuses the constraint (PR2017/PR2028).
set_option -use_sspi_as_gpio 1
run all
exit
