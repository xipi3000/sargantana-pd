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

add wave -noupdate /tb_module/module_inst/score_board_scalar_inst/mul
add wave -noupdate /tb_module/module_inst/score_board_scalar_inst/div
add wave -noupdate /tb_module/module_inst/score_board_scalar_inst/ready_mul_64_o
add wave -noupdate /tb_module/module_inst/score_board_scalar_inst/set_mul_32_i
add wave -noupdate /tb_module/module_inst/score_board_scalar_inst/set_mul_64_i
add wave -noupdate /tb_module/module_inst/alu_to_scalar_wb.valid
add wave -noupdate /tb_module/module_inst/alu_to_scalar_wb.result


add wave -noupdate /tb_module/tb_to_wb_o_1
add wave -noupdate /tb_module/tb_to_wb_o_2

add wave -noupdate -radix ascii /tb_module/tb_test_name
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {4271 ns} 0}
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
