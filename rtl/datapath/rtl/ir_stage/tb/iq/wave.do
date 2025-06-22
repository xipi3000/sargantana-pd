onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_instruction_queue/tb_clk_i
add wave -noupdate /tb_instruction_queue/tb_rstn_i
add wave -noupdate /tb_instruction_queue/tb_instruction_S_i
add wave -noupdate /tb_instruction_queue/instruction_S_o
add wave -noupdate /tb_instruction_queue/tb_read_head_S_i
add wave -noupdate /tb_instruction_queue/tb_empty_o
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/head
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/num
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/tail
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/instruction_buffer
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/write_enable_S
add wave -noupdate /tb_instruction_queue/instruction_queue_inst/read_enable_S
add wave -noupdate /tb_instruction_queue/tmp
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {270 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 281
configure wave -valuecolwidth 153
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
WaveRestoreZoom {258 ns} {283 ns}
