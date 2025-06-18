onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_module/tb_clk_i
add wave -noupdate /tb_module/tb_from_rr_i
add wave -noupdate /tb_module/tb_from_rr_i.instr.valid
add wave -noupdate /tb_module/tb_from_rr_i.instr.instr_type
add wave -noupdate /tb_module/tb_from_rr_i.instr.unit
add wave -noupdate /tb_module/tb_from_rr_i.data_rs1
add wave -noupdate /tb_module/tb_from_rr_i.data_rs2
add wave -noupdate /tb_module/module_inst/ready

add wave -noupdate /tb_module/module_inst/stall_int
add wave -noupdate /tb_module/module_inst/from_rr_i.instr.valid
add wave -noupdate /tb_module/module_inst/arith_instr.instr.valid
add wave -noupdate /tb_module/module_inst/arith_instr.data_rs1
add wave -noupdate /tb_module/module_inst/arith_instr.data_rs2
add wave -noupdate /tb_module/module_inst/mul_div_to_scalar_wb_o.result


add wave  -color yellow -noupdate /tb_module/module_inst/div_unit_inst/data_src1
add wave  -color yellow -noupdate /tb_module/module_inst/div_unit_inst/data_src2
add wave  -color yellow -noupdate /tb_module/module_inst/div_unit_inst/instruction_o.valid
add wave  -color yellow -noupdate /tb_module/module_inst/div_unit_inst/instruction_o.result
add wave -color yellow -noupdate /tb_module/module_inst/div_unit_inst/dividend_quotient_q 
add wave -color yellow -noupdate /tb_module/module_inst/div_unit_inst/remanent_q 
add wave -color yellow -noupdate /tb_module/module_inst/div_unit_inst/cycles_counter


*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/data_src1
*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/data_src2
*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/instruction_0_d.valid
*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/instruction_s1.valid
*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/instruction_s2.valid
*add wave  -color yellow -noupdate /tb_module/module_inst/mul_unit_inst/instruction_s2.result


add wave -color orange -noupdate /tb_module/module_inst/score_board_scalar_inst/mul
add wave -color orange -noupdate /tb_module/module_inst/score_board_scalar_inst/div
add wave -color orange -noupdate /tb_module/module_inst/score_board_scalar_inst/set_div_64_i
add wave -color orange -noupdate /tb_module/module_inst/score_board_scalar_inst/set_div_32_i
add wave -color orange -noupdate /tb_module/module_inst/score_board_scalar_inst/div_unit_sel_o


add wave -color purple -noupdate /tb_module/tb_to_wb_o_1.valid
add wave -color purple -noupdate /tb_module/tb_to_wb_o_1.result

add wave -color purple -noupdate /tb_module/tb_to_wb_o_2.result
add wave -color purple -noupdate /tb_module/tb_to_wb_o_2.valid



add wave -noupdate -radix ascii /tb_module/tb_test_name
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1073 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 258
configure wave -valuecolwidth 142
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {5201 ns} {5205 ns}
