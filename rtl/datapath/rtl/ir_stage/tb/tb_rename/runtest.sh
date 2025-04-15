VLOG_FLAGS=-svinputport=net 
CYCLES=-all
BASE_DIR="../../../../../.."
DRAC_FOLDER_RTL="${BASE_DIR}/rtl"
IF_STAGE="${BASE_DIR}/rtl/datapath/rtl/if_stage/rtl"
ID_STAGE="${BASE_DIR}/rtl/datapath/rtl/ir_stage/rtl"
ID_STAGE_TB="${BASE_DIR}/rtl/datapath/rtl/ir_stage/tb/tb_rename"
RR_STAGE="${BASE_DIR}/rtl/datapath/rtl/rr_stage/rtl"
DATAPATH="${BASE_DIR}/rtl/datapath/rtl"
INCLUDES="${BASE_DIR}/includes"

rm -rf lib_module

vlib lib_module 
vmap work $PWD/lib_module
vlog $VLOG_FLAGS +acc=rn +incdir+ $INCLUDES/riscv_pkg.sv $INCLUDES/fpuv_pkg.sv $INCLUDES/drac_pkg.sv\
 $ID_STAGE/rename_table.sv $ID_STAGE_TB/tb_rename_table.sv $ID_STAGE_TB/colors.vh
vmake lib_module/ > Makefile

if [ -z "$1" ]
then
      vsim work.tb_rename_table -do "view wave -new" -do "do wave.do" -do "run $CYCLES"
else
      vsim work.tb_rename_table $1 -do "run $CYCLES"
fi

