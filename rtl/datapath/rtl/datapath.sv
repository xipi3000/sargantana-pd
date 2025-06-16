/*
 * Copyright 2023 BSC*
 * *Barcelona Supercomputing Center (BSC)
 * 
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 * 
 * Licensed under the Solderpad Hardware License v 2.1 (the “License”); you
 * may not use this file except in compliance with the License, or, at your
 * option, the Apache License version 2.0. You may obtain a copy of the
 * License at
 * 
 * https://solderpad.org/licenses/SHL-2.1/
 * 
 * Unless required by applicable law or agreed to in writing, any work
 * distributed under the License is distributed on an “AS IS” BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

module datapath
    import drac_pkg::*;
    import riscv_pkg::*;
(
    input logic             clk_i,
    input logic             rstn_i,
    input addr_t            reset_addr_i,
    input logic             soft_rstn_i,
    // icache/dcache/CSR interface input
    input resp_icache_cpu_t resp_icache_cpu_i,
    input resp_dcache_cpu_t resp_dcache_cpu_i,
    input resp_csr_cpu_t    resp_csr_cpu_i,
    input [2:0]             csr_frm_i, 
    input [1:0]             csr_fs_i,  
    input [1:0]             csr_vs_i,  
    input logic             en_translation_i,
    input logic             en_ld_st_translation_i,
    input debug_in_t        debug_i,
    input [1:0]             csr_priv_lvl_i,
    input logic             req_icache_ready_i,
    input sew_t             sew_i,
    input tlb_cache_comm_t  dtlb_comm_i,
    // icache/dcache/CSR interface output
    output req_cpu_dcache_t req_cpu_dcache_o, 
    output req_cpu_icache_t req_cpu_icache_o,
    output req_cpu_csr_t    req_cpu_csr_o,
    output debug_out_t      debug_o,
    output cache_tlb_comm_t dtlb_comm_o,
    //--PMU   
    output to_PMU_t         pmu_flags_o
);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// SIGNAL DECLARATION                                                                           /////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

`ifdef SIM_COMMIT_LOG
    logic commit_valid[1:0];
    commit_data_t commit_data[1:0];
    addr_t store_addr;
    bus64_t store_data;
`endif

    bus64_t pc_if1, pc_if2, pc_id, pc_rr, pc_exe, pc_wb;
    logic valid_if1, valid_if2, valid_id, valid_rr, valid_exe, valid_wb;

    pipeline_ctrl_t [1:0] control_int_S;
    pipeline_flush_t flush_int;
    cu_if_t cu_if_int;
    addrPC_t pc_jump_if_int;
    addrPC_t pc_evec_q;
    addrPC_t pc_next_csr_q;

    
    // Pipelines stages data
    // Fetch
    if_1_if_2_stage_t stage_if_1_if_2_d;
    if_1_if_2_stage_t stage_if_1_if_2_q;
    if_id_stage_t stage_if_2_id_d; // this is the saving in the current cycle
    if_id_stage_t stage_if_2_id_q; // this is the next or output of reg
    logic invalidate_icache_int;
    logic invalidate_buffer_int;
    logic retry_fetch;
    
    // Decode
    id_ir_stage_t decoded_instr;
    id_ir_stage_t stored_instr_id_d;
    id_ir_stage_t stored_instr_id_q;
    id_ir_stage_t [1:0] selection_id_ir_S;

    id_cu_t id_cu_int;
    jal_id_if_t jal_id_if_int;
    
    logic src_select_id_ir_q;
    
    // Rename and free list
    id_ir_stage_t [NUM_SCALAR_INSTR:0] stage_iq_ir_q_S;
    id_ir_stage_t [NUM_SCALAR_INSTR:0] stage_iq_ir_q_0;
    id_ir_stage_t [NUM_SCALAR_INSTR:0] stage_iq_ir_q_1;
    id_ir_stage_t stage_ir_rr_d;
    ir_rr_stage_t_S stage_ir_rr_q_Ss;
    ir_rr_stage_t_S stage_stall_rr_q_Ss;
    ir_rr_stage_t_S stage_no_stall_rr_q_Ss;

    logic ir_scalar_rdy_src1_int;
    logic ir_scalar_rdy_src2_int;
    logic ir_fp_rdy_src1_int;
    logic ir_fp_rdy_src2_int;

    phreg_t ir_scalar_preg_src1_int;
    phreg_t ir_scalar_preg_src2_int;
    phreg_t ir_scalar_old_dst_int;
    phreg_t ir_fp_preg_src2_int;
    phreg_t ir_fp_preg_src1_int;
    phreg_t ir_fp_old_dst_int;

    logic do_checkpoint;
    logic do_recover;
    logic delete_checkpoint;
    logic out_of_checkpoints_rename;
    logic out_of_checkpoints_free_list;
    logic fp_out_of_checkpoints_rename;
    logic fp_out_of_checkpoints_free_list;

    logic is_iq_2_empty;

    logic free_list_empty;
    logic fp_free_list_empty;

    phreg_t [NUM_SCALAR_INSTR-1:0]  free_register_to_rename_S;
    phfreg_t fp_free_register_to_rename;

    checkpoint_ptr checkpoint_free_list;
    checkpoint_ptr checkpoint_rename;
    checkpoint_ptr fp_checkpoint_free_list;
    checkpoint_ptr fp_checkpoint_rename;

    logic src_select_ir_rr_q;

    ir_cu_t ir_cu_int;
    cu_ir_t cu_ir_int;

    reg_t [1:0] free_list_read_src1_int_S;

    // Read Registers
    rr_exe_instr_t_S stage_rr_exe_d_Ss;
    rr_exe_instr_t_S stage_rr_exe_q_Ss;

    bus64_t [NUM_SCALAR_INSTR-1:0] rr_data_scalar_src1_S;
    bus64_t [NUM_SCALAR_INSTR-1:0] rr_data_scalar_src2_S;
    bus64_t rr_data_fp_src1;
    bus64_t rr_data_fp_src2;

    logic [drac_pkg::NUM_SCALAR_WB-1:0] snoop_rr_rs1;
    logic [drac_pkg::NUM_SCALAR_WB-1:0] snoop_rr_rs2;
    logic [NUM_SCALAR_INSTR-1:0] snoop_rr_rdy1_S;
    logic [NUM_SCALAR_INSTR-1:0] snoop_rr_rdy2_S;
    logic snoop_rr_vrdy1;
    logic snoop_rr_vrdy2;
    logic snoop_rr_vrdym;

    logic [drac_pkg::NUM_FP_WB-1:0] snoop_rr_frs1;
    logic [drac_pkg::NUM_FP_WB-1:0] snoop_rr_frs2;
    logic [drac_pkg::NUM_FP_WB-1:0] snoop_rr_frs3;
    logic snoop_rr_frdy1;
    logic snoop_rr_frdy2;
    logic snoop_rr_frdy3;

    rr_cu_t rr_cu_int;
    cu_rr_t cu_rr_int;

    logic is_csr_int;
    reg_csr_addr_t csr_addr_int;
    exception_t [NUM_SCALAR_INSTR-1 : 0] ex_gl_in_int;

    bus64_t result_gl_out_int;
    reg_csr_addr_t csr_addr_gl_out_int;
    exception_t ex_gl_out_int;

    exception_t [NUM_SCALAR_INSTR] interrupt_ex_S;

    exception_t ex_from_exe_int;
    gl_index_t ex_from_exe_index_int;
    // Graduation List

    gl_instruction_t [NUM_SCALAR_INSTR-1:0] instruction_decode_gl_S;
    
    gl_wb_data_t [drac_pkg::NUM_SCALAR_WB-1:0] instruction_writeback_gl;
    gl_index_t       [drac_pkg::NUM_SCALAR_WB-1:0] gl_index;
    logic            [drac_pkg::NUM_SCALAR_WB-1:0] gl_valid;
    // FP
    gl_wb_data_t [drac_pkg::NUM_FP_WB-1:0] instruction_fp_writeback_gl;
    gl_index_t       [drac_pkg::NUM_FP_WB-1:0] gl_index_fp;
    logic            [drac_pkg::NUM_FP_WB-1:0] gl_valid_fp;

    gl_instruction_t [1:0] instruction_gl_commit; 
    
    // Exe
    rr_exe_instr_t_S selection_rr_exe_d_Ss;

    exe_cu_t exe_cu_int;
    exe_wb_scalar_instr_t [drac_pkg::NUM_SCALAR_WB-1:0] exe_to_wb_scalar;
    exe_wb_scalar_instr_t [drac_pkg::NUM_SCALAR_WB-1:0] wb_scalar;
    exe_wb_fp_instr_t [drac_pkg::NUM_FP_WB-1:0] exe_to_wb_fp;
    exe_wb_fp_instr_t [drac_pkg::NUM_FP_WB-1:0] wb_fp;

    bus64_t [NUM_SCALAR_WB-1:0] snoop_exe_data_rs1_S;
    bus64_t [NUM_SCALAR_WB-1:0] snoop_exe_data_rs2_S;
    logic   [NUM_SCALAR_WB-1:0] snoop_exe_rs1;
    logic   [NUM_SCALAR_WB-1:0] snoop_exe_rs2;
    logic snoop_exe_rdy1;
    logic snoop_exe_rdy2;
    logic pmu_exe_ready;

    bus64_t snoop_exe_data_frs1;
    bus64_t snoop_exe_data_frs2;
    bus64_t snoop_exe_data_frs3;
    logic   [drac_pkg::NUM_FP_WB-1:0] snoop_exe_frs1;
    logic   [drac_pkg::NUM_FP_WB-1:0] snoop_exe_frs2;
    logic   [drac_pkg::NUM_FP_WB-1:0] snoop_exe_frs3;
    logic snoop_exe_frdy1;
    logic snoop_exe_frdy2;
    logic snoop_exe_frdy3;

    bus64_t exe_data_rs1;
    bus64_t exe_data_rs2;
    bus64_t exe_data_frs1;
    bus64_t exe_data_frs2;
    bus64_t exe_data_frs3;
    rr_exe_instr_t_S reg_to_exe_Ss;
    rr_exe_instr_t [NUM_SCALAR_INSTR:0] reg_to_exe_S;
    // This addresses are fixed from lowrisc
    reg_addr_t io_base_addr;

    // codifies if the branch was correctly predicted 
    // this signal goes from exe stage to fetch stage
    logic correct_branch_pred;

    // WB->Commit
    wb_cu_t wb_cu_int;
    cu_wb_t cu_wb_int;
    
    exe_if_branch_pred_t exe_if_branch_pred_int;   

    // Commit signals
    commit_cu_t commit_cu_int;
    cu_commit_t cu_commit_int;
    logic commit_xcpt;
    bus64_t commit_xcpt_cause;
    logic [1:0] commit_store_or_amo_int;
    logic mem_commit_store_or_amo_int;
    
    //gl_instruction_t instruction_gl_commit_old_q;
    gl_instruction_t [1:0] instruction_to_commit;
    logic src_select_commit;
    exception_t exception_mem_commit_int;
    gl_index_t mem_gl_index_int;
    gl_index_t index_gl_commit;
    logic [1:0] retire_inst_gl;
    //gl_index_t index_gl_commit_old_q;

    //Br at WB
    addrPC_t branch_addr_result_wb;
    logic correct_branch_pred_wb;

    // CSR signals
    logic   csr_ena_int;

    // Data to write to RR from WB or CSR
    bus64_t [NUM_SCALAR_WB-1:0] data_wb_to_rr;
    bus64_t [NUM_SCALAR_WB-1:0] data_wb_to_exe;
    phreg_t [NUM_SCALAR_WB-1:0] write_paddr_rr;
    phreg_t [NUM_SCALAR_WB-1:0] write_paddr_exe;
    reg_t   [NUM_SCALAR_WB-1:0] write_vaddr;

    // Data to write to RR from WB or CSR
    bus64_t [drac_pkg::NUM_FP_WB-1:0] fp_data_wb_to_rr;
    bus64_t [drac_pkg::NUM_FP_WB-1:0] fp_data_wb_to_exe;
    phreg_t [drac_pkg::NUM_FP_WB-1:0] fp_write_paddr_rr;
    phreg_t [drac_pkg::NUM_FP_WB-1:0] fp_write_paddr_exe;
    reg_t   [drac_pkg::NUM_FP_WB-1:0] fp_write_vaddr;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// IO ADDRESS SPACE                                                                             /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Debug signals
    bus64_t    reg_wr_data;
    phreg_t    reg_wr_addr;
    phreg_t [NUM_SCALAR_INSTR-1:0]   reg_prd1_addr_S;
    // stall IF
    logic stall_if;
    logic miss_icache;
    `ifdef SIM_KONATA_DUMP
        bus64_t id_fetch;
    `endif

    // This addresses are fixed from lowrisc
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            io_base_addr <=  40'h0040000000;
        end else if(!soft_rstn_i) begin
            io_base_addr <=  40'h0040000000;
        end else begin 
            io_base_addr <= io_base_addr;
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// CONTROL UNIT                                                                                 /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Control Unit
    control_unit control_unit_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .miss_icache_i(miss_icache),
        .ready_icache_i(req_icache_ready_i),
        .id_cu_i(id_cu_int),
        .ir_cu_i(ir_cu_int),
        .cu_ir_o(cu_ir_int),
        .rr_cu_i(rr_cu_int),
        .cu_rr_o(cu_rr_int),
        .wb_cu_i(wb_cu_int),
        .cu_wb_o(cu_wb_int),
        .exe_cu_i(exe_cu_int),
        .csr_cu_i(resp_csr_cpu_i),
        .pipeline_ctrl_o(control_int),
        .pipeline_flush_o(flush_int),
        .cu_if_o(cu_if_int),
        .invalidate_icache_o(invalidate_icache_int),
        .invalidate_buffer_o(invalidate_buffer_int),
        .correct_branch_pred_exe_i(correct_branch_pred),
        .correct_branch_pred_wb_i(correct_branch_pred_wb),
        .debug_halt_i(debug_i.halt_valid),
        .debug_change_pc_i(debug_i.change_pc_valid),
        .debug_wr_valid_i(debug_i.reg_write_valid),
        .commit_cu_i(commit_cu_int),
        .cu_commit_o(cu_commit_int),
        .pmu_jump_misspred_o(pmu_flags_o.branch_miss)
    );

    // Combinational logic select the jump addr
    // from decode or wb 
    always_comb begin
        retry_fetch = 1'b0;
        if (control_int.sel_addr_if == SEL_JUMP_DEBUG) begin
            pc_jump_if_int = debug_i.change_pc_addr;
        end else if (control_int.sel_addr_if == SEL_JUMP_EXECUTION) begin
            pc_jump_if_int = branch_addr_result_wb;
        end else if (control_int.sel_addr_if == SEL_JUMP_CSR) begin
            pc_jump_if_int = pc_evec_q;
            retry_fetch = 1'b1;
        end else if (control_int.sel_addr_if == SEL_JUMP_CSR_RW) begin
            pc_jump_if_int = pc_next_csr_q;
            retry_fetch = 1'b1;   
        end else if (control_int.sel_addr_if == SEL_JUMP_DECODE) begin
            pc_jump_if_int = jal_id_if_int.jump_addr;
        end else begin
            pc_jump_if_int = 64'h0;
            `ifdef ASSERTIONS
                assert (1 == 0);
            `endif
        end
    end

    assign stall_if_1 = control_int.stall_if_1 || debug_i.halt_valid;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// FETCH                  STAGE                                                                 /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // IF Stage
    if_stage_1 if_stage_1_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .reset_addr_i(reset_addr_i),
        .stall_debug_i(debug_i.halt_valid),
        .stall_i(stall_if_1),
        .cu_if_i(cu_if_int),
        .invalidate_icache_i(invalidate_icache_int),
        .invalidate_buffer_i(invalidate_buffer_int),
        .en_translation_i(en_translation_i), 
        .pc_jump_i(pc_jump_if_int),
        .retry_fetch_i(retry_fetch),
        .req_cpu_icache_o(req_cpu_icache_o),
        .fetch_o(stage_if_1_if_2_d),
        `ifdef SIM_KONATA_DUMP
        .id_o(id_fetch),
        `endif
        .exe_if_branch_pred_i(exe_if_branch_pred_int)
    );

    // Register IF1 to IF2
    register #($bits(if_1_if_2_stage_t)) reg_if_1_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_if),
        .load_i(!control_int.stall_if_1),
        .input_i(stage_if_1_if_2_d),
        .output_o(stage_if_1_if_2_q)
    );

    if_stage_2 if_stage_2_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .fetch_i(stage_if_1_if_2_q),
        .stall_i(control_int.stall_if_2),
        .flush_i(flush_int.flush_if),
        .resp_icache_cpu_i(resp_icache_cpu_i),
        .fetch_o(stage_if_2_id_d),
        .stall_o(miss_icache)
    );

    // Register IF to ID
    register #($bits(if_id_stage_t)) reg_if_2_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_if),
    .load_i(!control_int.stall_if_2),
    .input_i(stage_if_2_id_d),
    .output_o(stage_if_2_id_q)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// DECODER                           STAGE                                                      /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ID Stage
    decoder id_decode_inst(
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),
        .stall_i        (control_int.stall_id),
        .flush_i        (flush_int.flush_id),
        .decode_i       (stage_if_2_id_q),
        .frm_i          (csr_frm_i),
        .csr_fs_i       (csr_fs_i), 
        .csr_vs_i       (csr_vs_i), 
        .decode_instr_o (decoded_instr),
        .jal_id_if_o    (jal_id_if_int)
    );

    // valid jal in decode
    assign id_cu_int.valid               = decoded_instr.instr.valid;
    assign id_cu_int.valid_jal           = jal_id_if_int.valid;
    assign id_cu_int.stall_csr_fence     = decoded_instr.instr.stall_csr_fence && decoded_instr.instr.valid;
    assign id_cu_int.predicted_as_branch = decoded_instr.instr.bpred.is_branch;
    assign id_cu_int.is_branch           = (decoded_instr.instr.instr_type == BLT)  ||
                                           (decoded_instr.instr.instr_type == BLTU) ||
                                           (decoded_instr.instr.instr_type == BGE)  ||
                                           (decoded_instr.instr.instr_type == BGEU) ||
                                           (decoded_instr.instr.instr_type == BEQ)  ||
                                           (decoded_instr.instr.instr_type == BNE)  ||
                                           (decoded_instr.instr.instr_type == JAL) ||
                                           (decoded_instr.instr.instr_type == JALR);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// INSTRUCTION QUEUE, FREE LIST AND RENAME               STAGE                                  /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    always_comb begin
        for (int i = 0; i<drac_pkg::NUM_SCALAR_INSTR; ++i) begin
            stored_instr_id_d_S[i] = (src_select_id_ir_q_S[i]) ? decoded_instr_S[i] : stored_instr_id_q_S[i];
            //TODO: CONFLICTS WITH DEBUG
            free_list_read_src1_int_S[i] = (debug_i.reg_read_valid  && debug_i.halt_valid)  ? debug_i.reg_read_write_addr : stage_iq_ir_q_S[i].instr.rs1;
        end
    end
    assign debug_o.reg_list_paddr = stage_no_stall_rr_q.prs1;

    // Register ID to IR when stall
    register #($bits(id_ir_stage_t)*(NUM_SCALAR_INSTR)) reg_id_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_id),
        .load_i(1'b1),
        .input_i(stored_instr_id_d_S),
        .output_o(stored_instr_id_q_S)
    );

    // Syncronus Mux to decide between actual decode or one cycle before
    always @(posedge clk_i) begin
        src_select_id_ir_q_S <= {!control_int_S[0].stall_id,!control_int[1].stall_id};
    end


    always_comb begin
         for (int i = 0; i<drac_pkg::NUM_SCALAR_INSTR; ++i) begin
            selection_id_ir_S[i] = (src_select_id_ir_q_S[i]) ? decoded_instr_S[i] : stored_instr_id_q_S[i],
         end

        if(selection_id_ir_S)


    end
    
        // Instruction Queue 
    instruction_queue instruction_queue_inst_1(
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),  
        .flush_i        (flush_int.flush_ir),  
        .instruction_S_i  ({selection_id_ir_S[0],
                           (selection_id_ir_S[1].instr.regfile_we & selection_id_ir_S[1].instr.mem_type == NOT_MEM) ? 'h0 : selection_id_ir_S[1]}), 
        .read_head_S_i    ({~control_int_S[0].stall_iq,
                        (is_iq_2_empty) ? ~control_int_S[0].stall_iq : 'h0}),
        .instruction_S_o  (stage_iq_ir_q_0),
        .full_o         (ir_cu_int.full_iq),
        .empty_o        ()
    );
    instruction_queue instruction_queue_inst_2(
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),  
        .flush_i        (flush_int.flush_ir),  
        .instruction_S_i  ({(selection_id_ir_S[1].instr.regfile_we & selection_id_ir_S[1].instr.mem_type == NOT_MEM) ?  selection_id_ir_S[1] : 'h0,
                        'h0
                        }), 
        .read_head_S_i    ({~control_int_S[1].stall_iq,'h0}),
        .instruction_S_o  (stage_iq_ir_q_1),
        .full_o         (ir_cu_int.full_iq),
        .empty_o        (is_iq_2_empty)
    );

    assign stage_iq_ir_q_S ={stage_iq_ir_q_0[0],  stage_iq_ir_q_0[1] == 'h0 ?  stage_iq_ir_q_1[0] : stage_iq_ir_q_0[1]};
    // Free List
    free_list free_list_inst(
        .clk_i                  (clk_i),
        .rstn_i                 (rstn_i),
        .read_head_S_i            ({stage_iq_ir_q_S[0].instr.regfile_we & stage_iq_ir_q_S[0].instr.valid & (stage_iq_ir_q_S[0].instr.rd != 'h0) & (~control_int_S[0].stall_ir) & (~control_int_S[0].stall_iq),      
                                    stage_iq_ir_q_S[1].instr.regfile_we & stage_iq_ir_q_S[1].instr.valid & (stage_iq_ir_q_S[1].instr.rd != 'h0) & (~control_int_S[1].stall_ir) & (~control_int_S[1].stall_iq)}),
        .add_free_register_S_i    (cu_ir_int.enable_commit_update),
        .free_register_S_i        ({instruction_to_commit[1].old_prd, instruction_to_commit[0].old_prd}),
        .do_checkpoint_i        (cu_ir_int.do_checkpoint),
        .do_recover_i           (cu_ir_int.do_recover),
        .delete_checkpoint_i    (cu_ir_int.delete_checkpoint),
        .recover_checkpoint_i   (cu_ir_int.recover_checkpoint),
        .commit_roll_back_i     (cu_ir_int.recover_commit),
        .new_register_S_o         (free_register_to_rename_S),
        .checkpoint_o           (checkpoint_free_list),
        .out_of_checkpoints_o   (out_of_checkpoints_free_list),
        .empty_o                (free_list_empty)
    );


//TODO: DO AN OR WITH THE INSTR [0] AND [1], SO IT DOESN'T MATTER WHERE IS IT 
    fp_free_list fp_free_list_inst(
        .clk_i                  (clk_i),
        .rstn_i                 (rstn_i),
        .read_head_i            (stage_iq_ir_q_S[0].instr.fregfile_we & stage_iq_ir_q_S[0].instr.valid & (~control_int.stall_ir) & (~control_int.stall_iq)),
        .add_free_register_i    (cu_ir_int.fp_enable_commit_update),
        .free_register_i        ({instruction_to_commit[1].old_fprd, instruction_to_commit[0].old_fprd}),
        .do_checkpoint_i        (cu_ir_int.do_checkpoint),
        .do_recover_i           (cu_ir_int.do_recover),
        .delete_checkpoint_i    (cu_ir_int.delete_checkpoint),
        .recover_checkpoint_i   (cu_ir_int.recover_checkpoint),
        .commit_roll_back_i     (cu_ir_int.recover_commit),
        .new_register_o         (fp_free_register_to_rename),
        .checkpoint_o           (fp_checkpoint_free_list),
        .out_of_checkpoints_o   (fp_out_of_checkpoints_free_list),
        .empty_o                (fp_free_list_empty)
    );

    // Rename Table
    rename_table rename_table_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .read_src1_S_i(free_list_read_src1_int_S),
        .read_src2_S_i({stage_iq_ir_q_S[0].instr.rs2,   stage_iq_ir_q_S[1].instr.rs2}),
        .old_dst_S_i({stage_iq_ir_q_S[0].instr.rd , stage_iq_ir_q_S[1].instr.rd}),
        .write_dst_S_i({stage_iq_ir_q_S[0].instr.regfile_we & stage_iq_ir_q_S[0].instr.valid & (~control_int_S[0].stall_ir) & (~control_int_S[0].stall_iq),
                        stage_iq_ir_q_S[1].instr.regfile_we & stage_iq_ir_q_S[1].instr.valid & (~control_int_S[1].stall_ir) & (~control_int_S[1].stall_iq)}),
        .new_dst_S_i(free_register_to_rename_S),
        .use_rs1_S_i({stage_iq_ir_q_S[0].instr.use_rs1 | (debug_i.reg_read_valid  && debug_i.halt_valid),
                     stage_iq_ir_q_S[1].instr.use_rs1 | (debug_i.reg_read_valid  && debug_i.halt_valid)}),
        .use_rs2_S_i({stage_iq_ir_q_S[0].instr.use_rs2,
                        stage_iq_ir_q_S[1].instr.use_rs2}),
        .ready_i(cu_rr_int.write_enable),
        .vaddr_i(write_vaddr),
        .paddr_i(write_paddr_rr),
        .do_checkpoint_i(cu_ir_int.do_checkpoint),
        .do_recover_i(cu_ir_int.do_recover),
        .delete_checkpoint_i(cu_ir_int.delete_checkpoint),
        .recover_checkpoint_i(cu_ir_int.recover_checkpoint),
        .recover_commit_i(cu_ir_int.recover_commit), 
        .commit_old_dst_i({instruction_to_commit[1].rd, instruction_to_commit[0].rd}),    
        .commit_write_dst_i(cu_ir_int.enable_commit_update),  
        .commit_new_dst_i({instruction_to_commit[1].prd, instruction_to_commit[0].prd}),
        .src1_S_o(stage_no_stall_rr_q_Ss.prs1),
        .rdy1_S_o(stage_no_stall_rr_q_Ss.rdy1),
        .src2_S_o(stage_no_stall_rr_q_Ss.prs2),
        .rdy2_S_o(stage_no_stall_rr_q_Ss.rdy2),
        .old_dst_S_o(stage_no_stall_rr_q_Ss.old_prd),
        .checkpoint_o(checkpoint_rename),
        .out_of_checkpoints_o(out_of_checkpoints_rename)
    );

    // FP Rename Table 
    fp_rename_table fp_rename_table_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .read_src1_i(stage_iq_ir_q_S[0].instr.rs1),
        .read_src2_i(stage_iq_ir_q_S[0].instr.rs2),
        .read_src3_i(stage_iq_ir_q_S[0].instr.rs3),
        .old_dst_i(stage_iq_ir_q_S[0].instr.rd),
        .write_dst_i(stage_iq_ir_q_S[0].instr.fregfile_we & stage_iq_ir_q_S[0].instr.valid & (~control_int.stall_ir) & (~control_int.stall_iq)),
        .new_dst_i(fp_free_register_to_rename),
        .use_fs1_i(stage_iq_ir_q_S[0].instr.use_fs1),
        .use_fs2_i(stage_iq_ir_q_S[0].instr.use_fs2),
        .use_fs3_i(stage_iq_ir_q_S[0].instr.use_fs3),
        .ready_i(cu_rr_int.fwrite_enable),
        .vaddr_i(fp_write_vaddr), // WB
        .paddr_i(fp_write_paddr_rr), // WB
        .do_checkpoint_i(cu_ir_int.do_checkpoint),
        .do_recover_i(cu_ir_int.do_recover),
        .delete_checkpoint_i(cu_ir_int.delete_checkpoint),
        .recover_checkpoint_i(cu_ir_int.recover_checkpoint),
        .recover_commit_i(cu_ir_int.recover_commit),
        .commit_old_dst_i({instruction_to_commit[1].rd, instruction_to_commit[0].rd}),
        .commit_write_dst_i(cu_ir_int.fp_enable_commit_update),
        .commit_new_dst_i({instruction_to_commit[1].fprd, instruction_to_commit[0].fprd}),
        .src1_o(stage_no_stall_rr_q_Ss.fprs1),
        .rdy1_o(stage_no_stall_rr_q_Ss.frdy1),
        .src2_o(stage_no_stall_rr_q_Ss.fprs2),
        .rdy2_o(stage_no_stall_rr_q_Ss.frdy2),
        .src3_o(stage_no_stall_rr_q_Ss.fprs3),
        .rdy3_o(stage_no_stall_rr_q_Ss.frdy3),
        .old_dst_o(stage_no_stall_rr_q_Ss.old_fprd),
        .checkpoint_o(fp_checkpoint_rename),
        .out_of_checkpoints_o(fp_out_of_checkpoints_rename)
    );
    
    // Check two structures output the same
    /*always @(posedge clk_i) assert (out_of_checkpoints_rename == out_of_checkpoints_free_list);
    always @(posedge clk_i) assert (checkpoint_rename == checkpoint_free_list);
    always @(posedge clk_i) assert (simd_out_of_checkpoints_rename == simd_out_of_checkpoints_free_list);
    always @(posedge clk_i) assert (simd_checkpoint_rename == simd_checkpoint_free_list);
    always @(posedge clk_i) assert (fp_out_of_checkpoints_rename == fp_out_of_checkpoints_free_list);
    always @(posedge clk_i) assert (fp_checkpoint_rename == fp_checkpoint_free_list); */

    assign stage_no_stall_rr_q_Ss.chkp = checkpoint_rename;

    // Signals for Control Unit
    assign ir_cu_int.valid                   = stage_iq_ir_q.instr.valid;
    assign ir_cu_int.empty_free_list         = free_list_empty;
    assign ir_cu_int.out_of_checkpoints      = out_of_checkpoints_rename;
    assign ir_cu_int.fp_out_of_checkpoints   = fp_out_of_checkpoints_rename;
    assign ir_cu_int.is_branch               = (stage_iq_ir_q.instr.instr_type == BLT)  ||
                                               (stage_iq_ir_q.instr.instr_type == BLTU) ||
                                               (stage_iq_ir_q.instr.instr_type == BGE)  ||
                                               (stage_iq_ir_q.instr.instr_type == BGEU) ||
                                               (stage_iq_ir_q.instr.instr_type == BEQ)  ||
                                               (stage_iq_ir_q.instr.instr_type == BNE)  ||
                                               (stage_iq_ir_q.instr.instr_type == JALR);
    always_comb begin
        stage_ir_rr_d_S = stage_iq_ir_q_S;
        for (int i; i< NUM_SCALAR_INSTR; i++) begin
            stage_ir_rr_d_S[i].instr.valid = stage_iq_ir_q_S[i].instr.valid & (~control_int.stall_iq); 
        end
    end 
    // Register IR to RR
    register #($bits(id_ir_stage_t) * (NUM_SCALAR_INSTR) + $bits(phreg_t) + $bits(phreg_t) + $bits(logic)) reg_ir_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_ir),
        .load_i(!control_int.stall_ir),
        .input_i({stage_ir_rr_d_S,free_register_to_rename_S, fp_free_register_to_rename, cu_ir_int.do_checkpoint}),
        .output_o({stage_no_stall_rr_q_Ss.instr,stage_no_stall_rr_q_Ss.ex,stage_no_stall_rr_q_Ss.prd,stage_no_stall_rr_q_Ss.fprd,stage_no_stall_rr_q_Ss.checkpoint_done})
    );

    // Second IR to RR. To store rename in case of stall
    register #($bits(ir_rr_stage_t_S)) reg_rename_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_ir),
        .load_i(1'b1), // This register is always storing a one cycle old copy of reg_ir_inst and the renaming.
        .input_i(stage_ir_rr_q_Ss),
        .output_o(stage_stall_rr_q_Ss)
    );

    // Syncronus Mux to decide between actual Rename or one cycle before Rename
    always @(posedge clk_i) begin
        src_select_ir_rr_q <= !control_int.stall_ir;
    end
    always_comb begin
        for(int i=0; i< NUM_SCALAR_INSTR; i++) begin
            if (src_select_ir_rr_q) begin
                stage_ir_rr_q_Ss.instr = stage_no_stall_rr_q_Ss.instr;
                stage_ir_rr_q_Ss.ex = stage_no_stall_rr_q_Ss.ex;
                stage_ir_rr_q_Ss.prd = stage_no_stall_rr_q_Ss.prd;
                stage_ir_rr_q_Ss.prs1 = stage_no_stall_rr_q_Ss.prs1;
                stage_ir_rr_q_Ss.prs2 = stage_no_stall_rr_q_Ss.prs2;
                stage_ir_rr_q_Ss.rdy1 = stage_no_stall_rr_q_Ss.rdy1 | snoop_rr_rdy1_S;
                stage_ir_rr_q_Ss.rdy2 = stage_no_stall_rr_q_Ss.rdy2 | snoop_rr_rdy2_S;
                stage_ir_rr_q_Ss.old_prd = stage_no_stall_rr_q_Ss.old_prd;
                stage_ir_rr_q_Ss.fprd = stage_no_stall_rr_q_Ss.fprd;
                stage_ir_rr_q_Ss.fprs1 = stage_no_stall_rr_q_Ss.fprs1;
                stage_ir_rr_q_Ss.fprs2 = stage_no_stall_rr_q_Ss.fprs2;
                stage_ir_rr_q_Ss.fprs3 = stage_no_stall_rr_q_Ss.fprs3;
                stage_ir_rr_q_Ss.frdy1 = stage_no_stall_rr_q_Ss.frdy1 | snoop_rr_frdy1;
                stage_ir_rr_q_Ss.frdy2 = stage_no_stall_rr_q_Ss.frdy2 | snoop_rr_frdy2;
                stage_ir_rr_q_Ss.frdy3 = stage_no_stall_rr_q_Ss.frdy3 | snoop_rr_frdy3;
                stage_ir_rr_q_Ss.old_fprd = stage_no_stall_rr_q_Ss.old_fprd;
                stage_ir_rr_q_Ss.chkp = stage_no_stall_rr_q_Ss.chkp;
                stage_ir_rr_q_Ss.checkpoint_done = stage_no_stall_rr_q_Ss.checkpoint_done;
            end else begin
                stage_ir_rr_q_Ss.instr = stage_stall_rr_q_Ss.instr;
                stage_ir_rr_q_Ss.ex = stage_stall_rr_q_Ss.ex;
                stage_ir_rr_q_Ss.prd = stage_stall_rr_q_Ss.prd;
                stage_ir_rr_q_Ss.prs1 = stage_stall_rr_q_Ss.prs1;
                stage_ir_rr_q_Ss.prs2 = stage_stall_rr_q_Ss.prs2;
                stage_ir_rr_q_Ss.rdy1 = stage_stall_rr_q_Ss.rdy1 | snoop_rr_rdy1_S;
                stage_ir_rr_q_Ss.rdy2 = stage_stall_rr_q_Ss.rdy2 | snoop_rr_rdy2_S;
                stage_ir_rr_q_Ss.old_prd = stage_stall_rr_q_Ss.old_prd;
                stage_ir_rr_q_Ss.fprd = stage_stall_rr_q_Ss.fprd;
                stage_ir_rr_q_Ss.fprs1 = stage_stall_rr_q_Ss.fprs1;
                stage_ir_rr_q_Ss.fprs2 = stage_stall_rr_q_Ss.fprs2;
                stage_ir_rr_q_Ss.fprs3 = stage_stall_rr_q_Ss.fprs3;
                stage_ir_rr_q_Ss.frdy1 = stage_stall_rr_q_Ss.frdy1 | snoop_rr_frdy1;
                stage_ir_rr_q_Ss.frdy2 = stage_stall_rr_q_Ss.frdy2 | snoop_rr_frdy2;
                stage_ir_rr_q_Ss.frdy3 = stage_stall_rr_q_Ss.frdy3 | snoop_rr_frdy3;
                stage_ir_rr_q_Ss.old_fprd = stage_stall_rr_q_Ss.old_fprd;
                stage_ir_rr_q_Ss.chkp = stage_stall_rr_q_Ss.chkp;
                stage_ir_rr_q_Ss.checkpoint_done = stage_stall_rr_q_Ss.checkpoint_done;
            end
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// GRADUATION LIST AND READ REGISTER  STAGE                                                     /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    always_comb begin

        for(int i =0; i<NUM_SCALAR_INSTR; i++) begin
            instruction_decode_gl_S[i].valid                  = stage_ir_rr_q_Ss.instr[i][i].valid & (~control_int.stall_rr);
            instruction_decode_gl_S[i].instr[i]_type             = stage_ir_rr_q_Ss.instr[i].instr_type;
            instruction_decode_gl_S[i].rd                     = stage_ir_rr_q_Ss.instr[i].rd;
            instruction_decode_gl_S[i].rs1                    = stage_ir_rr_q_Ss.instr[i].rs1;
            instruction_decode_gl_S[i].pc                     = stage_ir_rr_q_Ss.instr[i].pc;
            instruction_decode_gl_S[i].stall_csr_fence        = stage_ir_rr_q_Ss.instr[i].stall_csr_fence;
            instruction_decode_gl_S[i].old_prd                = stage_ir_rr_q_Ss.old_prd[i];
            instruction_decode_gl_S[i].old_fprd               = stage_ir_rr_q_Ss.old_fprd;
            instruction_decode_gl_S[i].prd                    = stage_ir_rr_q_Ss.prd[i];
            instruction_decode_gl_S[i].fprd                   = stage_ir_rr_q_Ss.fprd;
            instruction_decode_gl_S[i].regfile_we             = stage_ir_rr_q_Ss.instr[i].regfile_we;
            instruction_decode_gl_S[i].fregfile_we            = stage_ir_rr_q_Ss.instr[i].fregfile_we;
            `ifdef SIM_KONATA_DUMP
                instruction_decode_gl_S[i].id                 = stage_ir_rr_q_Ss.instr[i].id;
            `endif
            `ifdef SIM_COMMIT_LOG
                instruction_decode_gl_S[i].inst               = stage_ir_rr_q_Ss.instr[i].inst;
                instruction_decode_gl_S[i].exception = !stage_ir_rr_q_Ss.ex[i].valid && resp_csr_cpu_i.csr_interrupt ?  interrupt_ex_S[i] : stage_ir_rr_q_Ss.ex[i];
            `endif
                instruction_decode_gl_S[i].fp_status              = '0;
            instruction_decode_gl_S[i].mem_type               = stage_ir_rr_q_Ss.instr[i].mem_type;
            interrupt_ex_S[i].valid = resp_csr_cpu_i.csr_interrupt;
            interrupt_ex_S[i].cause = exception_cause_t'(resp_csr_cpu_i.csr_interrupt_cause);
            interrupt_ex_S[i].origin = 64'b0;
            instruction_decode_gl_S[i].ex_valid = stage_ir_rr_q_Ss.ex[i].valid | resp_csr_cpu_i.csr_interrupt;

            ex_gl_in_int_S[i] = !stage_ir_rr_q_Ss.ex[i].valid && resp_csr_cpu_i.csr_interrupt ? interrupt_ex_S[i] : stage_ir_rr_q_Ss.ex[i] ;
        end
    end



  

    // selecting the exception source, interrupt or exception from the front-end
    


    assign is_csr_int =(stage_ir_rr_q_Ss.instr.instr_type == ECALL ||
                        stage_ir_rr_q_Ss.instr.instr_type == SRET   ||
                        stage_ir_rr_q_Ss.instr.instr_type == MRET   ||
                        stage_ir_rr_q_Ss.instr.instr_type == URET   ||
                        stage_ir_rr_q_Ss.instr.instr_type == WFI    ||
                        stage_ir_rr_q_Ss.instr.instr_type == EBREAK ||
                        stage_ir_rr_q_Ss.instr.instr_type == FENCE  || 
                        stage_ir_rr_q_Ss.instr.instr_type == SFENCE_VMA || 
                        stage_ir_rr_q_Ss.instr.instr_type == FENCE_I|| 
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRW  ||
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRS  ||
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRC  ||
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRWI ||
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRSI ||
                        stage_ir_rr_q_Ss.instr.instr_type == CSRRCI);
    assign csr_addr_int = stage_ir_rr_q_Ss.instr.imm[CSR_ADDR_SIZE-1:0];
    

    graduation_list graduation_list_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .instruction_S_i(instruction_decode_gl_S),
        .is_csr_i(is_csr_int),
        .csr_addr_i(csr_addr_int),
        //TODO: CHECK THIS EXEPTION BECAUSE IT SHOULD BE ONE FOR INSTR
        .ex_i(ex_gl_in_int_S),
        .read_head_S_i(retire_inst_gl),
        .instruction_writeback_i(gl_index),
        .instruction_writeback_enable_i(gl_valid),
        .instruction_writeback_data_i(instruction_writeback_gl),
        .instruction_fp_writeback_i(gl_index_fp),
        .instruction_fp_writeback_enable_i(gl_valid_fp),
        .instruction_fp_writeback_data_i(instruction_fp_writeback_gl),
        .ex_from_exe_index_i(ex_from_exe_index_int),
        .ex_from_exe_i(ex_from_exe_int),
        .flush_i(cu_wb_int.flush_gl),
        .flush_index_i(cu_wb_int.flush_gl_index),
        .flush_commit_i(cu_commit_int.flush_gl_commit),
        //TODO: CHECK THIS ALSO
        .assigned_gl_entry_o(stage_rr_exe_d_Ss.gl_index),
        .instruction_o(instruction_gl_commit),
        .commit_gl_entry_o(index_gl_commit),
        .full_o(rr_cu_int.gl_full),
        .empty_o(debug_o.reg_backend_empty),
        .csr_addr_o(csr_addr_gl_out_int),
        .result_o(result_gl_out_int),
        .exception_o(ex_gl_out_int)
    );

    always_comb begin
        snoop_rr_rdy1_S = 1'b0;
        snoop_rr_rdy2_S = 1'b0;
        snoop_rr_frdy1 = 1'b0;
        snoop_rr_frdy2 = 1'b0;
        snoop_rr_frdy3 = 1'b0;
        
        for (int i = 0; i<NUM_SCALAR_WB; ++i) begin
            for(int j =0; j<NUM_SCALAR_INSTR; j++)begin
                snoop_rr_rdy1_S[j] |= cu_rr_int.snoop_enable[i] & (write_paddr_exe[i] == stage_ir_rr_q_Ss.prs1[j]) & (stage_ir_rr_q_Ss.instr[j].rs1!= 0);
                snoop_rr_rdy2_S[j] |= cu_rr_int.snoop_enable[i] & (write_paddr_exe[i] == stage_ir_rr_q_Ss.prs2[j]) & (stage_ir_rr_q_Ss.instr[j].rs2!= 0);
            end
        end

        for (int i = 0; i<drac_pkg::NUM_FP_WB; ++i) begin
            snoop_rr_frdy1 |= cu_rr_int.fwrite_enable[i] & (fp_write_paddr_exe[i] == stage_ir_rr_q_Ss.fprs1);
            snoop_rr_frdy2 |= cu_rr_int.fwrite_enable[i] & (fp_write_paddr_exe[i] == stage_ir_rr_q_Ss.fprs2);
            snoop_rr_frdy3 |= cu_rr_int.fwrite_enable[i] & (fp_write_paddr_exe[i] == stage_ir_rr_q_Ss.fprs3);
        end

        for(int i =0; i<NUM_SCALAR_INSTR; i++) begin
            reg_prd1_addr_S[i]  = (debug_i.reg_p_read_valid  && debug_i.halt_valid)  ? debug_i.reg_read_write_paddr : stage_ir_rr_q_Ss.prs1[i];
        end
    end


    
    // RR Stage
    regfile regfile_inst(
        .clk_i (clk_i),

        .write_enable_i(cu_rr_int.write_enable),
        .write_addr_i(write_paddr_rr),
        .write_data_i(data_wb_to_rr),
        
        .read_addr1_S_i(reg_prd1_addr_S),
        .read_addr2_S_i(stage_ir_rr_q_Ss.prs2),
        .read_data1_S_o(rr_data_scalar_src1_S),
        .read_data2_S_o(rr_data_scalar_src2_S)
    );

    // RR Stage
    regfile_fp regfile_fp_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),

        .write_enable_i(cu_rr_int.fwrite_enable),
        .write_addr_i(fp_write_paddr_rr),
        .write_data_i(fp_data_wb_to_rr),
        
        .read_addr1_i(stage_ir_rr_q_Ss.fprs1),
        .read_addr2_i(stage_ir_rr_q_Ss.fprs2),
        .read_addr3_i(stage_ir_rr_q_Ss.fprs3),
        .read_data1_o(rr_data_fp_src1),
        .read_data2_o(rr_data_fp_src2),
        .read_data3_o(stage_rr_exe_d_Ss.data_rs3)
    );

    // Decide from which Regfile to Read FP
    always_comb begin : read_src
        for(int i=0; i<NUM_SCALAR_INSTR;i++) begin
            if (stage_ir_rr_q_Ss.instr[i].use_fs1) begin
                stage_rr_exe_d_Ss.data_rs1[i] = rr_data_fp_src1;
            end else begin // From Scalar
                stage_rr_exe_d_Ss.data_rs1[i] = rr_data_scalar_src1_S[i];
            end
            if (stage_ir_rr_q_Ss.instr[i].use_fs2) begin 
                stage_rr_exe_d_Ss.data_rs2[i] = rr_data_fp_src2;
            end else begin // From Scalar
                stage_rr_exe_d_Ss.data_rs2[i] = rr_data_scalar_src2_S[i];
            end
        end
    end

    always_comb begin
        stage_rr_exe_d_Ss.instr = stage_ir_rr_q_Ss.instr;
        for(int i=0; i<NUM_SCALAR_INSTR;i++) begin
            //TODO:CHEKC resp_csr_cpu_i
            stage_rr_exe_d_Ss.instr[i].valid = stage_ir_rr_q_Ss.instr[i].valid && !(stage_ir_rr_q_Ss.instr[i].ex_valid | resp_csr_cpu_i.csr_interrupt);
            stage_rr_exe_d_Ss.instr[i].ex_valid = stage_ir_rr_q_Ss.instr[i].ex_valid | resp_csr_cpu_i.csr_interrupt;
        end
    end
    assign stage_rr_exe_d_Ss.prd = stage_ir_rr_q_Ss.prd;
    assign stage_rr_exe_d_Ss.prs1 = stage_ir_rr_q_Ss.prs1;
    assign stage_rr_exe_d_Ss.prs2 = stage_ir_rr_q_Ss.prs2;
    assign stage_rr_exe_d_Ss.rdy1 = stage_ir_rr_q_Ss.rdy1;
    assign stage_rr_exe_d_Ss.rdy2 = stage_ir_rr_q_Ss.rdy2;
    assign stage_rr_exe_d_Ss.old_prd = stage_ir_rr_q_Ss.old_prd;
    assign stage_rr_exe_d_Ss.fprd = stage_ir_rr_q_Ss.fprd;
    assign stage_rr_exe_d_Ss.fprs1 = stage_ir_rr_q_Ss.fprs1;
    assign stage_rr_exe_d_Ss.fprs2 = stage_ir_rr_q_Ss.fprs2;
    assign stage_rr_exe_d_Ss.fprs3 = stage_ir_rr_q_Ss.fprs3;
    assign stage_rr_exe_d_Ss.frdy1 = stage_ir_rr_q_Ss.frdy1;
    assign stage_rr_exe_d_Ss.frdy2 = stage_ir_rr_q_Ss.frdy2;
    assign stage_rr_exe_d_Ss.frdy3 = stage_ir_rr_q_Ss.frdy3;
    assign stage_rr_exe_d_Ss.old_fprd = stage_ir_rr_q_Ss.old_fprd;
    assign stage_rr_exe_d_Ss.chkp = stage_ir_rr_q_Ss.chkp;
    assign stage_rr_exe_d_Ss.checkpoint_done = stage_ir_rr_q_Ss.checkpoint_done;


    assign selection_rr_exe_d_Ss = (control_int.stall_rr) ? reg_to_exe_Ss : stage_rr_exe_d_Ss;

    // Register RR to EXE
    register #($bits(stage_rr_exe_d_Ss)) reg_rr_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_rr),
        .load_i(1'b1),
        .input_i(selection_rr_exe_d_Ss),
        .output_o(stage_rr_exe_q_Ss)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// EXECUTION STAGE                                                                              /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        snoop_exe_data_rs1_S = {64'b0 ,64'b0};
        snoop_exe_data_rs2_S = {64'b0 ,64'b0};
        snoop_exe_data_frs1 = 64'b0;
        snoop_exe_data_frs2 = 64'b0;
        snoop_exe_data_frs3 = 64'b0;

        for (int i = 0; i<drac_pkg::NUM_SCALAR_WB; ++i) begin
            snoop_exe_rs1[i] = cu_rr_int.snoop_enable[i] & (write_paddr_exe[i] == stage_rr_exe_q_Ss.prs1[i]) & (stage_rr_exe_q_Ss.instr[i].rs1 != 0);
            snoop_exe_rs2[i] = cu_rr_int.snoop_enable[i] & (write_paddr_exe[i] == stage_rr_exe_q_Ss.prs2[i]) & (stage_rr_exe_q_Ss.instr[i].rs2 != 0);
            snoop_exe_data_rs1_S[i] = snoop_exe_rs1[i] ? data_wb_to_exe[i] : 64'b0;
            snoop_exe_data_rs2_S[i] = snoop_exe_rs2[i] ? data_wb_to_exe[i] : 64'b0;
        end

        for (int i = 0; i<drac_pkg::NUM_FP_WB; ++i) begin
            snoop_exe_frs1[i] = cu_rr_int.fsnoop_enable[i] & (fp_write_paddr_exe[i] == stage_rr_exe_q_Ss.fprs1);
            snoop_exe_frs2[i] = cu_rr_int.fsnoop_enable[i] & (fp_write_paddr_exe[i] == stage_rr_exe_q_Ss.fprs2);
            snoop_exe_frs3[i] = cu_rr_int.fsnoop_enable[i] & (fp_write_paddr_exe[i] == stage_rr_exe_q_Ss.fprs3);
            snoop_exe_data_frs1 |= snoop_exe_frs1[i] ? fp_data_wb_to_exe[i] : 64'b0;
            snoop_exe_data_frs2 |= snoop_exe_frs2[i] ? fp_data_wb_to_exe[i] : 64'b0;
            snoop_exe_data_frs3 |= snoop_exe_frs3[i] ? fp_data_wb_to_exe[i] : 64'b0;
        end

        //snoop_exe_rdy1 = |snoop_exe_rs1;
        //snoop_exe_rdy2 = |snoop_exe_rs2;
        for (int i=0; i<NUM_SCALAR_INSTR; ++i) begin
        exe_data_rs1[i] = snoop_exe_rs1[i] ? (snoop_exe_data_rs1_S[i]) : stage_rr_exe_q_Ss.data_rs1[i];
        exe_data_rs2[i] = snoop_exe_rs2[i] ? (snoop_exe_data_rs2_S[i]) : stage_rr_exe_q_Ss.data_rs2[i];
        
        end
        snoop_exe_frdy1 = |snoop_exe_frs1;
        snoop_exe_frdy2 = |snoop_exe_frs2;
        snoop_exe_frdy3 = |snoop_exe_frs3;
        //TODO: SELECT THE WAY WITH THE FP FOR REAL
        exe_data_frs1 = snoop_exe_frdy1 ? (snoop_exe_data_frs1) : stage_rr_exe_q_Ss.data_rs1[0];
        exe_data_frs2 = snoop_exe_frdy2 ? (snoop_exe_data_frs2) : stage_rr_exe_q_Ss.data_rs2[0];
        exe_data_frs3 = snoop_exe_frdy3 ? (snoop_exe_data_frs3) : stage_rr_exe_q_Ss.data_rs3;
    end

    assign reg_to_exe_Ss.instr = stage_rr_exe_q_Ss.instr;
    assign reg_to_exe_Ss.data_rs1[0] = (stage_rr_exe_q_Ss.instr[0].use_fs1) ? exe_data_frs1 : exe_data_rs1[0];
    assign reg_to_exe_Ss.data_rs2[0] = (stage_rr_exe_q_Ss.instr[0].use_fs2) ? exe_data_frs2 : exe_data_rs2[0];
    assign reg_to_exe_Ss.data_rs1[1] = (stage_rr_exe_q_Ss.instr[1].use_fs1) ? exe_data_frs1 : exe_data_rs1[1];
    assign reg_to_exe_Ss.data_rs2[1] = (stage_rr_exe_q_Ss.instr[1].use_fs2) ? exe_data_frs2 : exe_data_rs2[1];
    assign reg_to_exe_Ss.data_rs3 = exe_data_frs3;
    
    assign reg_to_exe_Ss.prs1 = stage_rr_exe_q_Ss.prs1;
    assign reg_to_exe_Ss.rdy1 = snoop_exe_rdy1 | stage_rr_exe_q_Ss.rdy1;
    assign reg_to_exe_Ss.prs2 = stage_rr_exe_q_Ss.prs2;
    assign reg_to_exe_Ss.rdy2 = snoop_exe_rdy2 | stage_rr_exe_q_Ss.rdy2;
    assign reg_to_exe_Ss.prd = stage_rr_exe_q_Ss.prd;
    assign reg_to_exe_Ss.old_prd = stage_rr_exe_q_Ss.old_prd;
    
    assign reg_to_exe_Ss.fprs1 = stage_rr_exe_q_Ss.fprs1;
    assign reg_to_exe_Ss.frdy1 = snoop_exe_frdy1 | stage_rr_exe_q_Ss.frdy1;
    assign reg_to_exe_Ss.fprs2 = stage_rr_exe_q_Ss.fprs2;
    assign reg_to_exe_Ss.frdy2 = snoop_exe_frdy2 | stage_rr_exe_q_Ss.frdy2;
    assign reg_to_exe_Ss.fprs3 = stage_rr_exe_q_Ss.fprs3;
    assign reg_to_exe_Ss.frdy3 = snoop_exe_frdy3 | stage_rr_exe_q_Ss.frdy3;
    assign reg_to_exe_Ss.fprd = stage_rr_exe_q_Ss.fprd;
    assign reg_to_exe_Ss.old_fprd = stage_rr_exe_q_Ss.old_fprd;

    assign reg_to_exe_Ss.checkpoint_done = stage_rr_exe_q_Ss.checkpoint_done;
    assign reg_to_exe_Ss.chkp = stage_rr_exe_q_Ss.chkp;
    assign reg_to_exe_Ss.gl_index = stage_rr_exe_q_Ss.gl_index;

    always_comb begin
        for(int i =0; i<NUM_SCALAR_INSTR; i++) begin
            reg_to_exe_S[i].instr = stage_rr_exe_q_Ss.instr[i];
            reg_to_exe_S[i].data_rs1 = (stage_rr_exe_q_Ss.instr[i].use_fs1) ? exe_data_frs1 : exe_data_rs1[i];
            reg_to_exe_S[i].data_rs2 = (stage_rr_exe_q_Ss.instr[i].use_fs2) ? exe_data_frs2 : exe_data_rs2[i];
            reg_to_exe_S[i]s.data_rs3 = exe_data_frs3;
            
            reg_to_exe_S[i].prs1 = stage_rr_exe_q_Ss.prs1[i];
            reg_to_exe_S[i].rdy1 = snoop_exe_rdy1 | stage_rr_exe_q_Ss.rdy1[i];
            reg_to_exe_S[i].prs2 = stage_rr_exe_q_Ss.prs2[i];
            reg_to_exe_S[i].rdy2 = snoop_exe_rdy2 | stage_rr_exe_q_Ss.rdy2[i];
            reg_to_exe_S[i].prd = stage_rr_exe_q_Ss.prd[i];
            reg_to_exe_S[i].old_prd = stage_rr_exe_q_Ss.old_prd[i];
            
            reg_to_exe_S[i].fprs1 = stage_rr_exe_q_Ss.fprs1;
            reg_to_exe_S[i].frdy1 = snoop_exe_frdy1 | stage_rr_exe_q_Ss.frdy1;
            reg_to_exe_S[i].fprs2 = stage_rr_exe_q_Ss.fprs2;
            reg_to_exe_S[i].frdy2 = snoop_exe_frdy2 | stage_rr_exe_q_Ss.frdy2;
            reg_to_exe_S[i].fprs3 = stage_rr_exe_q_Ss.fprs3;
            reg_to_exe_S[i].frdy3 = snoop_exe_frdy3 | stage_rr_exe_q_Ss.frdy3;
            reg_to_exe_S[i].fprd = stage_rr_exe_q_Ss.fprd;
            reg_to_exe_S[i].old_fprd = stage_rr_exe_q_Ss.old_fprd;

            reg_to_exe_S[i].checkpoint_done = stage_rr_exe_q_Ss.checkpoint_done;
            reg_to_exe_S[i].chkp = stage_rr_exe_q_Ss.chkp;
            reg_to_exe_S[i].gl_index = stage_rr_exe_q_Ss.gl_index;
        end
    end




    exe_stage exe_stage_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),

        .kill_i(flush_int.kill_exe),

        .en_ld_st_translation_i(en_ld_st_translation_i),

        .from_rr_i(reg_to_exe_S[0]),
        .sew_i(sew_i),
        
        .resp_dcache_cpu_i(resp_dcache_cpu_i),
        .io_base_addr_i(io_base_addr),
        .flush_i(flush_int.flush_exe),
        .commit_store_or_amo_i(commit_store_or_amo_int),
        .commit_store_or_amo_gl_idx_i(commit_cu_int.gl_index),
        .dtlb_comm_i(dtlb_comm_i),
        .dtlb_comm_o(dtlb_comm_o),
        .priv_lvl_i(csr_priv_lvl_i),

        `ifdef SIM_COMMIT_LOG
        .store_addr_o(store_addr),
        .store_data_o(store_data),
        `endif
    
        .exe_if_branch_pred_o(exe_if_branch_pred_int),
        .correct_branch_pred_o(correct_branch_pred),
    
        .arith_to_scalar_wb_o(exe_to_wb_scalar[0]),
        .mem_to_scalar_wb_o(exe_to_wb_scalar[1]),
        .mul_div_to_scalar_wb_o(exe_to_wb_scalar[3]),

        .fp_to_scalar_wb_o(exe_to_wb_scalar[2]),

        .mem_to_fp_wb_o(exe_to_wb_fp[1]),
        .fp_to_wb_o(exe_to_wb_fp[0]),
        .exe_cu_o(exe_cu_int),

        .mem_commit_stall_o(mem_commit_stall_int),
        .mem_store_or_amo_o(mem_commit_store_or_amo_int),
        .mem_gl_index_o(mem_gl_index_int),
        .exception_mem_commit_o(exception_mem_commit_int),
        .ex_gl_o(ex_from_exe_int),
        .ex_gl_index_o(ex_from_exe_index_int),

        .req_cpu_dcache_o(req_cpu_dcache_o),

        //PMU Neiel-Leyva
        .pmu_is_branch_o          (pmu_flags_o.is_branch),      
        .pmu_branch_taken_o       (pmu_flags_o.branch_taken),   
        .pmu_stall_mem_o          (pmu_flags_o.stall_wb),
        .pmu_exe_ready_o          (pmu_exe_ready),
        .pmu_struct_depend_stall_o(pmu_flags_o.struct_depend),
        .pmu_load_after_store_o   (pmu_flags_o.stall_rr)
    );

    exe_stage_red exe_stage_red_inst(

        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .kill_i(flush_int.kill_exe),
        .from_rr_i(reg_to_exe_S[1]),
        .flush_i(flush_int.flush_exe),
    
        .exe_if_branch_pred_o(exe_if_branch_pred_int),
        .correct_branch_pred_o(correct_branch_pred),
    
        .arith_to_scalar_wb_o(exe_to_wb_scalar[1]),
        .mul_div_to_scalar_wb_o(exe_to_wb_scalar[1]),
        .exe_cu_o(exe_cu_int),
        //PMU Neiel-Leyva
        .pmu_is_branch_o          (pmu_flags_o.is_branch),      
        .pmu_branch_taken_o       (pmu_flags_o.branch_taken),   
        .pmu_stall_mem_o          (pmu_flags_o.stall_wb),
        .pmu_exe_ready_o          (pmu_exe_ready),
        .pmu_struct_depend_stall_o(pmu_flags_o.struct_depend),
    );




    register #( (NUM_SCALAR_WB) * $bits(exe_wb_scalar_instr_t) + (NUM_FP_WB) * $bits(exe_wb_fp_instr_t)) reg_exe_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_exe),
        .load_i(!control_int.stall_exe),
        .input_i({exe_to_wb_scalar[0], exe_to_wb_scalar[1], exe_to_wb_scalar[2], exe_to_wb_scalar[3], exe_to_wb_fp[0], exe_to_wb_fp[1]}),
        .output_o({wb_scalar[0], wb_scalar[1], wb_scalar[2], wb_scalar[3], wb_fp[0], wb_fp[1]})
    );

    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            branch_addr_result_wb <=  40'h0040000000;
            correct_branch_pred_wb <=  1'b1;
        end else if(!control_int.stall_exe) begin
            branch_addr_result_wb <=  exe_if_branch_pred_int.branch_addr_result_exe;
            correct_branch_pred_wb <=  correct_branch_pred;
        end else begin 
            branch_addr_result_wb <= branch_addr_result_wb;
            correct_branch_pred_wb <= correct_branch_pred_wb;
        end
    end 

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// WRITE BACK STAGE                                                                             /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


    assign wb_amo_int = wb_scalar[1].mem_type == AMO;

    //WB data for the bypasses (the CSRs should not be bypassed)
    always_comb begin
        for (int i = 0; i<NUM_SCALAR_WB; ++i) begin
            //Graduation list writeback arrays
            if (i == 1) begin
                gl_valid[i] = wb_scalar[i].valid & ~wb_amo_int;
                gl_index[i] = wb_scalar[i].gl_index;
                instruction_writeback_gl[i].csr_addr = wb_scalar[i].csr_addr;
                instruction_writeback_gl[i].exception = wb_scalar[i].ex;
                instruction_writeback_gl[i].result   = wb_scalar[i].result;
                instruction_writeback_gl[i].fp_status = wb_scalar[i].fp_status;
                `ifdef SIM_COMMIT_LOG
                instruction_writeback_gl[i].addr    = wb_scalar[i].addr;
                `endif
            end else begin
                gl_valid[i] = wb_scalar[i].valid;
                gl_index[i] = wb_scalar[i].gl_index;
                instruction_writeback_gl[i].csr_addr = wb_scalar[i].csr_addr;
                instruction_writeback_gl[i].exception = wb_scalar[i].ex;
                instruction_writeback_gl[i].result   = wb_scalar[i].result;
                instruction_writeback_gl[i].fp_status = wb_scalar[i].fp_status;
                `ifdef SIM_COMMIT_LOG
                instruction_writeback_gl[i].addr    = wb_scalar[i].addr;
                `endif
            end

            // Write data regfile from WB or from Commit (CSR)
            // CSR are exclusive with the rest of instrucitons. Therefor, there are no conflicts
            if (i == 0) begin
                // Change the data of write port 0 with dbg ring data
                wb_cu_int.write_enable[i] = wb_scalar[i].regfile_we;
                data_wb_to_exe[i] = wb_scalar[i].result;
                write_paddr_exe[i] = wb_scalar[i].prd;
                write_vaddr[i] = (commit_cu_int.write_enable) ? instruction_to_commit[0].rd :
                                  wb_scalar[i].rd;
                wb_cu_int.snoop_enable[i] = wb_scalar[i].regfile_we;
            end else begin
                data_wb_to_exe[i]  = wb_scalar[i].result;
                write_paddr_exe[i] = wb_scalar[i].prd;
                write_vaddr[i]     = wb_scalar[i].rd;
                wb_cu_int.write_enable[i] = wb_scalar[i].regfile_we;
                wb_cu_int.snoop_enable[i] = wb_scalar[i].regfile_we;
            end
            wb_cu_int.valid[i]        = wb_scalar[i].valid;
        end
        wb_cu_int.change_pc_ena = wb_scalar[0].change_pc_ena;
        
        for (int i = 0; i<NUM_FP_WB; ++i) begin
            //Graduation list writeback arrays
            gl_valid_fp[i] = wb_fp[i].valid  & wb_fp[i].regfile_we;
            gl_index_fp[i] = wb_fp[i].gl_index;
            instruction_fp_writeback_gl[i].csr_addr  = wb_fp[i].csr_addr;
            instruction_fp_writeback_gl[i].exception = wb_fp[i].ex;
            instruction_fp_writeback_gl[i].result    = wb_fp[i].result;
            instruction_fp_writeback_gl[i].fp_status = wb_fp[i].fp_status;
            fp_data_wb_to_exe[i]  = wb_fp[i].result;
            fp_write_paddr_exe[i] = wb_fp[i].fprd;
            fp_write_vaddr[i]     = wb_fp[i].rd;
            wb_cu_int.fwrite_enable[i] = wb_fp[i].regfile_we;
            wb_cu_int.fsnoop_enable[i] = wb_fp[i].regfile_we;
            wb_cu_int.fvalid[i]        = wb_fp[i].valid;
            `ifdef SIM_COMMIT_LOG
            instruction_fp_writeback_gl[i].addr      = wb_fp[i].addr;
            `endif
        end

        wb_cu_int.checkpoint_done = wb_scalar[0].checkpoint_done;
        wb_cu_int.chkp = wb_scalar[0].chkp;
        wb_cu_int.gl_index = wb_scalar[0].gl_index;

    end


    // WB data to RR
    always_comb begin
        for (int i = 0; i<NUM_SCALAR_WB; ++i) begin
            if (i == 0) begin
                // Change the data of write port 0 with dbg ring data
                if (debug_i.reg_write_valid && debug_i.halt_valid) begin
                    data_wb_to_rr[i] = debug_i.reg_write_data;
                    write_paddr_rr[i] = debug_i.reg_read_write_paddr;
                end else begin
                    data_wb_to_rr[i] = (commit_cu_int.write_enable) ? resp_csr_cpu_i.csr_rw_rdata : wb_scalar[i].result;
                    write_paddr_rr[i] = (commit_cu_int.write_enable) ? instruction_to_commit[0].prd : wb_scalar[i].prd;
                end
            end else begin
                data_wb_to_rr[i] = wb_scalar[i].result;
                write_paddr_rr[i] = wb_scalar[i].prd;
            end
        end

        for (int i = 0; i<NUM_FP_WB; ++i) begin
            fp_data_wb_to_rr[i]  = wb_fp[i].result;
            fp_write_paddr_rr[i] = wb_fp[i].fprd;
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// COMMIT STAGE                                                                                 /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    assign instruction_to_commit = instruction_gl_commit;
    assign commit_cu_int.gl_index = (commit_store_or_amo_int[0]) ? index_gl_commit : index_gl_commit + 1'b1;

    csr_interface csr_interface_inst
    (
        .commit_xcpt_i              (commit_xcpt),
        .result_gl_i                (result_gl_out_int),
        .csr_addr_gl_i              (csr_addr_gl_out_int),
        .instruction_to_commit_i    (instruction_to_commit),
        .stall_exe_i                (control_int.stall_exe),
        .commit_store_or_amo_i      (commit_store_or_amo_int[0]),
        .mem_commit_stall_i         (commit_cu_int.stall_commit),
        .exception_mem_commit_i     (exception_mem_commit_int),
        .exception_gl_i             (ex_gl_out_int),
        .csr_ena_int_o              (csr_ena_int),
        .req_cpu_csr_o              (req_cpu_csr_o),
        .retire_inst_o              (retire_inst_gl)
    );

    // Delay the PC_EVEC treatment one cycle
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            pc_evec_q <= 'b0;
            pc_next_csr_q <= 'b0;
        end else begin 
            pc_evec_q <= resp_csr_cpu_i.csr_evec;
            pc_next_csr_q <= instruction_to_commit[0].pc + 64'h4;
        end
    end

    // if there is an exception that can be from:
    // the instruction itself or the interrupt
    assign commit_xcpt = (~commit_store_or_amo_int[0])? ex_gl_out_int.valid & instruction_to_commit[0].ex_valid : exception_mem_commit_int.valid;
    assign commit_xcpt_cause = (~commit_store_or_amo_int[0])? ex_gl_out_int.cause : exception_mem_commit_int.cause;

    // Control Unit From Commit
    assign commit_cu_int.valid = instruction_to_commit[0].valid;
    assign commit_cu_int.regfile_we = {instruction_to_commit[1].regfile_we,instruction_to_commit[0].regfile_we};
    assign commit_cu_int.fregfile_we = {instruction_to_commit[1].fregfile_we,instruction_to_commit[0].fregfile_we};
    assign commit_cu_int.csr_enable = csr_ena_int;
    assign commit_cu_int.stall_csr_fence = instruction_to_commit[0].stall_csr_fence && instruction_to_commit[0].valid;
    assign commit_cu_int.xcpt = commit_xcpt;

    // tell cu that ecall was taken
    assign commit_cu_int.ecall_taken = (instruction_to_commit[0].instr_type == ECALL  ||
                                        instruction_to_commit[0].instr_type == MRTS   ||
                                        instruction_to_commit[0].instr_type == EBREAK );

    // tell cu that there is a fence or fence_i
    assign commit_cu_int.fence = (instruction_to_commit[0].instr_type == FENCE_I || 
                                  instruction_to_commit[0].instr_type == FENCE || 
                                  instruction_to_commit[0].instr_type == SFENCE_VMA);
    // tell cu there is a fence i to flush the icache
    assign commit_cu_int.fence_i = (instruction_to_commit[0].instr_type == FENCE_I || 
                                    instruction_to_commit[0].instr_type == SFENCE_VMA);

    // tell cu that commit needs to write there is a fence
    assign commit_cu_int.write_enable = instruction_to_commit[0].valid &
                                        (instruction_to_commit[0].instr_type == CSRRW  ||
                                         instruction_to_commit[0].instr_type == CSRRS  ||
                                         instruction_to_commit[0].instr_type == CSRRC  ||
                                         instruction_to_commit[0].instr_type == CSRRWI ||
                                         instruction_to_commit[0].instr_type == CSRRSI ||
                                         instruction_to_commit[0].instr_type == CSRRCI ||
                                         instruction_to_commit[0].instr_type == VSETVL ||
                                         instruction_to_commit[0].instr_type == VSETVLI);

    assign commit_store_or_amo_int[0] = (((instruction_to_commit[0].mem_type == STORE) || 
                                        (instruction_to_commit[0].mem_type == AMO)) && !instruction_to_commit[0].ex_valid);
    assign commit_store_or_amo_int[1] = (((instruction_to_commit[1].mem_type == STORE) || 
                                        (instruction_to_commit[1].mem_type == AMO)) && !instruction_to_commit[1].ex_valid);
        
    assign commit_cu_int.stall_commit = mem_commit_stall_int | (commit_store_or_amo_int[0] & ((commit_cu_int.gl_index != mem_gl_index_int) | !mem_commit_store_or_amo_int));
    assign commit_cu_int.retire = retire_inst_gl;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// DEBUG SIGNALS                                                                                /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

`ifdef SIM_COMMIT_LOG
    // Debug signals
    always_comb begin 
        for (int i=0; i<2; i++) begin

            commit_valid[i] = retire_inst_gl[i];

            commit_data[i].pc              = (instruction_to_commit[i].valid) ? instruction_to_commit[i].pc : 64'b0;
            commit_data[i].dst             = instruction_to_commit[i].rd;
            commit_data[i].fdst            = instruction_to_commit[i].rd;
            commit_data[i].reg_wr_valid    = instruction_to_commit[i].regfile_we && instruction_to_commit[i].rd != 5'b0;
            commit_data[i].freg_wr_valid   = instruction_to_commit[i].fregfile_we && commit_valid[i];
            commit_data[i].csr_wr_valid    =
                (instruction_to_commit[i].instr_type inside {CSRRW, CSRRWI} ) ||
                (instruction_to_commit[i].instr_type inside {CSRRS, CSRRC, CSRRSI, CSRRCI} && instruction_to_commit[i].rs1 != 5'b0);
            commit_data[i].csr_dst         = instruction_to_commit[i].csr_addr;
            commit_data[i].csr_data        = instruction_to_commit[i].result;
            commit_data[i].inst            = instruction_to_commit[i].inst;
            commit_data[i].sew             = sew_i;
            commit_data[i].xcpt            = commit_xcpt;
            commit_data[i].xcpt_cause      = commit_xcpt_cause;
            commit_data[i].csr_priv_lvl    = csr_priv_lvl_i;
            commit_data[i].csr_rw_data     = req_cpu_csr_o.csr_rw_data;
            commit_data[i].csr_xcpt        = resp_csr_cpu_i.csr_exception;
            commit_data[i].csr_xcpt_cause  = resp_csr_cpu_i.csr_exception_cause;
            commit_data[i].csr_tval        = resp_csr_cpu_i.csr_tval;
            commit_data[i].mem_type        = instruction_to_commit[i].mem_type;
            commit_data[i].mem_addr        = instruction_to_commit[i].addr;
            commit_data[i].fflags_wr_valid = instruction_to_commit[i].fp_status != 0;

            if (i==0) begin
                if(instruction_to_commit[0].valid) begin
                    if (commit_cu_int.write_enable) begin
                        commit_data[0].data = resp_csr_cpu_i.csr_rw_rdata;
                    end else if (commit_store_or_amo_int[0] & (commit_cu_int.gl_index == mem_gl_index_int)) begin
                        commit_data[0].data = instruction_to_commit[0].mem_type == STORE ? store_data : exe_to_wb_scalar[1].result;
                        commit_data[0].mem_addr = instruction_to_commit[0].mem_type == STORE ? store_addr : exe_to_wb_scalar[1].addr;
                    end else begin
                        commit_data[0].data = instruction_to_commit[0].result;
                    end
                end else begin
                    commit_data[0].data = 128'b0;
                end
            end else begin
                if(instruction_to_commit[i].valid) begin
                    commit_data[i].data = instruction_to_commit[i].result;
                end else begin
                    commit_data[i].data = 128'b0;
                end
            end
        end
    end


    // Module that generates the signature of the core to compare with spike
    logic commit_store_int, is_commit_store_valid;
    assign commit_store_int = instruction_to_commit[0].mem_type == STORE;
    assign is_commit_store_valid = instruction_to_commit[0].valid && !commit_cu_int.stall_commit && 
                                    commit_store_int && (commit_cu_int.gl_index == mem_gl_index_int);

    `ifdef SIM_COMMIT_LOG_DPI    
    commit_log_behav commit_log
    (
        .clk(clk_i),
        .rst(rstn_i),
        .commit_valid_i(commit_valid),
        .commit_data_i(commit_data)
    );
    `endif
`endif

`ifdef SIM_KONATA_DUMP
    konata_dump_behav konata_dump
    (
        .clk(clk_i),
        .rst(rstn_i),
        .if1_valid(valid_if1),
        .if1_id(id_fetch), 
        .if1_stall(control_int.stall_if_1),
        .if1_flush(flush_int.flush_if),

        .if2_valid(valid_if2),
        .if2_id(stage_if_2_id_d.id),
        .if2_stall(control_int.stall_if_2),
        .if2_flush(flush_int.flush_if),

        .id_valid(valid_id),
        .id_inst(stage_if_2_id_q.inst),
        .id_pc(pc_id),
        .id_id(stage_if_2_id_q.id),
        .id_stall(control_int.stall_id),
        .id_flush(flush_int.flush_id),

        .ir_valid(stage_iq_ir_q.instr.valid),
        .ir_id(stage_iq_ir_q.instr.id),
        .ir_stall(control_int.stall_ir),
        .ir_flush(flush_int.flush_ir),

        .rr_valid(valid_rr),
        .rr_id(stage_ir_rr_q_Ss.instr.id),
        .rr_stall(control_int.stall_rr),
        .rr_flush(flush_int.flush_rr),

        .exe_valid(valid_exe),
        .exe_id(stage_rr_exe_q_Ss.instr.id),
        .exe_stall(control_int.stall_exe),
        .exe_flush(flush_int.flush_exe),
        .exe_unit(reg_to_exe_Ss.instr.unit),

        .wb1_valid(wb_scalar[0].valid),
        .wb1_id(wb_scalar[0].id),

        .wb2_valid(wb_scalar[1].valid),
        .wb2_id(wb_scalar[1].id),

        .wb_store_valid(is_commit_store_valid),
        .wb_srore_id(instruction_to_commit[0].id),
        // Scalar 
        .wb3_valid(wb_scalar[2].valid),
        .wb3_id(wb_scalar[2].id),
        // Mult writeback
        .wb4_valid(wb_scalar[3].valid),
        .wb4_id(wb_scalar[3].id),
        // FP 1
        .wb1_fp_valid(wb_fp[0].valid),
        .wb1_fp_id(wb_fp[0].id),
        // FP 2
        .wb2_fp_valid(wb_fp[1].valid),
        .wb2_fp_id(wb_fp[1].id)
    );
`endif

        // PCcommit_freg_we
    assign pc_if1  = stage_if_1_if_2_d.pc_inst;
    assign pc_if2  = stage_if_2_id_d.pc_inst;
    assign pc_id  = (valid_id)  ? decoded_instr.instr.pc : 64'b0;
    assign pc_rr  = (valid_rr)  ? stage_rr_exe_d_Ss.instr.pc : 64'b0;
    assign pc_exe = (valid_exe) ? stage_rr_exe_q_Ss.instr.pc : 64'b0;
    assign pc_wb = (valid_wb) ? wb_scalar[0].pc : 64'b0;
    
        // Valid
    assign valid_if1  = stage_if_1_if_2_d.valid;
    assign valid_if2  = stage_if_2_id_d.valid;
    assign valid_id  = decoded_instr.instr.valid;
    assign valid_rr  = stage_rr_exe_d_Ss.instr.valid;
    assign valid_exe = stage_rr_exe_q_Ss.instr.valid;
    assign valid_wb = wb_scalar[0].valid;

    // Debug Ring signals Output
    // PC
    assign debug_o.pc_fetch = pc_if1[39:0];
    assign debug_o.pc_dec   = pc_id[39:0];
    assign debug_o.pc_rr    = pc_rr[39:0];
    assign debug_o.pc_exe   = pc_exe[39:0];
    assign debug_o.pc_wb    = pc_wb[39:0];
    // Write-back signals
    assign debug_o.wb_valid_1 = wb_scalar[0].valid;
    assign debug_o.wb_reg_addr_1 = wb_scalar[0].rd;
    assign debug_o.wb_reg_we_1 = wb_scalar[0].regfile_we;
    assign debug_o.wb_valid_2 = wb_scalar[1].valid;
    assign debug_o.wb_reg_addr_2 = wb_scalar[1].rd;
    assign debug_o.wb_reg_we_2 = wb_scalar[1].regfile_we;
    // Register File read 
    assign debug_o.reg_read_data = stage_rr_exe_d_Ss.data_rs1;


    //PMU
    assign pmu_flags_o.stall_if        = resp_csr_cpu_i.csr_stall ;
    
    assign pmu_flags_o.stall_id        = control_int.stall_id || ~decoded_instr.instr.valid;
    assign pmu_flags_o.stall_exe       = control_int.stall_exe || ~reg_to_exe_Ss.instr.valid;
    assign pmu_flags_o.load_store      = (~commit_cu_int.stall_commit) && (commit_store_or_amo_int[0] || instruction_to_commit[0].mem_type == LOAD);
    assign pmu_flags_o.data_depend     = ~pmu_exe_ready && ~pmu_flags_o.stall_exe;
    assign pmu_flags_o.grad_list_full  = rr_cu_int.gl_full && ~resp_csr_cpu_i.csr_stall && ~exe_cu_int.stall;
    assign pmu_flags_o.free_list_empty = free_list_empty && ~rr_cu_int.gl_full && ~resp_csr_cpu_i.csr_stall && ~exe_cu_int.stall;

    /*
    (* keep="TRUE" *) (* mark_debug="TRUE" *) gl_instruction_t [1:0] instruction_to_commit_reg;
    (* keep="TRUE" *) (* mark_debug="TRUE" *) commit_cu_t commit_cu_int_reg;
    (* keep="TRUE" *) (* mark_debug="TRUE" *) cu_ir_t cu_ir_int_reg;
    always_ff @(posedge clk_i) 
    begin
        instruction_to_commit_reg <= instruction_to_commit;
        commit_cu_int_reg <= commit_cu_int;
        cu_ir_int_reg <= cu_ir_int;
    end*/

endmodule
