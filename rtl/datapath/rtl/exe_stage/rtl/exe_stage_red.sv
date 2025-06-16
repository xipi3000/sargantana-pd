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

module exe_stage 
    import drac_pkg::*;
    import riscv_pkg::*;
(
    input logic                         clk_i,
    input logic                         rstn_i,
    input logic                         kill_i,
    input logic                         flush_i,

    // INPUTS
    input rr_exe_instr_t                from_rr_i,
    // OUTPUTS
    output exe_wb_scalar_instr_t        arith_to_scalar_wb_o,
    output exe_wb_scalar_instr_t        mul_div_to_scalar_wb_o,
    output exe_cu_t                     exe_cu_o,

    output logic                        correct_branch_pred_o,  // Decides if the branch prediction was correct  
    output exe_if_branch_pred_t         exe_if_branch_pred_o,   // Branch prediction (taken, target) and result (take, target)
    //--PMU
    output logic                        pmu_is_branch_o,
    output logic                        pmu_branch_taken_o,    
    output logic                        pmu_exe_ready_o,
    output logic                        pmu_struct_depend_stall_o,
);

// Declarations
bus64_t rs1_data_def;
bus_simd_t rs2_data_def;

rr_exe_arith_instr_t arith_instr;

exe_wb_scalar_instr_t alu_to_scalar_wb;
exe_wb_scalar_instr_t mul_to_scalar_wb;
exe_wb_scalar_instr_t div_to_scalar_wb;
exe_wb_scalar_instr_t branch_to_scalar_wb;

logic stall_int;




logic ready;
logic set_mul_32_inst;
logic set_mul_64_inst;
logic set_div_32_inst;
logic set_div_64_inst;
logic ready_1cycle_inst;
logic ready_2cycle_inst;
logic ready_mul_32_inst; 
logic ready_mul_64_inst;
logic ready_div_32_inst;

logic div_unit_sel;
logic ready_div_unit;

logic set_vmul_32_inst;
logic set_vmul_64_inst;
logic ready_vec_1cycle_inst;
logic is_vmul;
logic is_vred;


/* qnn_bseg start */

  bus64_t bs_rs1_mul_i  ;
  bus64_t bs_rs2_mul_i  ;
  logic   bs_mul_valid_i;
  bus64_t bs_rd_mul_o   ;
  logic   bs_mul_valid_o;

/* qnn_bseg end */

// Bypasses
`ifdef ASSERTIONS
    always @(posedge clk_i) begin
        if(from_rr_i.prs1 == 0)
            assert rs1_data_def==0;
        if(from_rr_i.prs2 == 0)
            assert rs2_data_def==0;
    end
`endif

// Select rs2 from imm to avoid bypasses
assign rs1_data_def = from_rr_i.instr.use_pc ? from_rr_i.instr.pc : from_rr_i.data_rs1;
assign rs2_data_def = from_rr_i.instr.use_imm ? from_rr_i.instr.imm : from_rr_i.data_rs2;

score_board_scalar score_board_scalar_inst(
    .clk_i            (clk_i),
    .rstn_i           (rstn_i),
    .flush_i          (flush_i),
    .set_mul_32_i     (set_mul_32_inst),               
    .set_mul_64_i     (set_mul_64_inst),               
    .set_div_32_i     (set_div_32_inst),               
    .set_div_64_i     (set_div_64_inst),
    .ready_1cycle_o   (ready_1cycle_inst),
    .ready_mul_32_o   (ready_mul_32_inst),
    .ready_mul_64_o   (ready_mul_64_inst),
    .ready_div_32_o   (ready_div_32_inst),
    .div_unit_sel_o   (div_unit_sel),
    .ready_div_unit_o (ready_div_unit)
);

assign ready = from_rr_i.instr.valid & ( (from_rr_i.rdy1 | from_rr_i.instr.use_pc) 
                                     & (from_rr_i.rdy2 | from_rr_i.instr.use_imm) 
                                     & (from_rr_i.frdy1) & (from_rr_i.frdy2) & (from_rr_i.frdy3));

always_comb begin
    arith_instr.data_rs1            = rs1_data_def;
    arith_instr.data_rs2            = rs2_data_def;
    arith_instr.prs1                = from_rr_i.prs1;
    arith_instr.rdy1                = from_rr_i.rdy1;
    arith_instr.prs2                = from_rr_i.prs2;
    arith_instr.rdy2                = from_rr_i.rdy2;
    arith_instr.prd                 = from_rr_i.prd;
    arith_instr.old_prd             = from_rr_i.old_prd;
    arith_instr.checkpoint_done     = from_rr_i.checkpoint_done;
    arith_instr.chkp                = from_rr_i.chkp;
    arith_instr.gl_index            = from_rr_i.gl_index;



    if (stall_int || kill_i) begin
        arith_instr.instr   = '0;
     
    end else begin
        arith_instr.instr   = from_rr_i.instr;

    end


end

alu alu_inst (
    .instruction_i  (arith_instr),
    .instruction_o  (alu_to_scalar_wb)
);

  mul_unit mul_unit_inst (
    .clk_i         (clk_i           ),
    .rstn_i        (rstn_i          ),
    .flush_mul_i   (flush_i         ),
    .instruction_i (arith_instr     ),
    .instruction_o (mul_to_scalar_wb)
  );

div_unit div_unit_inst (
    .clk_i          (clk_i),
    .rstn_i         (rstn_i),
    .flush_div_i    (flush_i),
    .div_unit_sel_i (div_unit_sel),
    .instruction_i  (arith_instr),
    .instruction_o  (div_to_scalar_wb)
);

branch_unit branch_unit_inst (
    .instruction_i      (arith_instr),
    .instruction_o      (branch_to_scalar_wb)
);


always_comb begin


    if (mul_to_scalar_wb.valid) begin
        mul_div_to_scalar_wb_o = mul_to_scalar_wb;
    end else if (div_to_scalar_wb.valid) begin
        mul_div_to_scalar_wb_o = div_to_scalar_wb;
    end else begin
        mul_div_to_scalar_wb_o = 'h0;
    end
    
    if (alu_to_scalar_wb.valid) begin
        arith_to_scalar_wb_o = alu_to_scalar_wb;
    end else if (branch_to_scalar_wb.valid) begin
        arith_to_scalar_wb_o = branch_to_scalar_wb;
    end else begin
        arith_to_scalar_wb_o = 'h0;
    end

end

always_comb begin
    stall_int = 1'b0;
    set_div_32_inst = 1'b0;
    set_div_64_inst = 1'b0;
    set_mul_32_inst = 1'b0;
    set_mul_64_inst = 1'b0;
    set_vmul_32_inst = 1'b0;
    set_vmul_64_inst = 1'b0;
    if (from_rr_i.instr.valid && !kill_i) begin
	    if (from_rr_i.instr.unit == UNIT_DIV & from_rr_i.instr.op_32) begin
            stall_int = ~ready | ~ready_div_32_inst | ~ready_div_unit;
            set_div_32_inst = ready & ready_div_32_inst & ready_div_unit;
        end
        else if (from_rr_i.instr.unit == UNIT_DIV & ~from_rr_i.instr.op_32) begin
            stall_int = ~ready | ~ready_div_unit;
            set_div_64_inst = ready & ready_div_unit;
        end
        else if (from_rr_i.instr.unit == UNIT_MUL & from_rr_i.instr.op_32) begin
            stall_int = ~ready | ~ready_mul_32_inst;
            set_mul_32_inst = ready & ready_mul_32_inst;
        end
        else if (from_rr_i.instr.unit == UNIT_MUL & ~from_rr_i.instr.op_32) begin
            stall_int = ~ready | ~ready_mul_64_inst;
            set_mul_64_inst = ready & ready_mul_64_inst;
        end
        else if (from_rr_i.instr.unit == UNIT_ALU | from_rr_i.instr.unit == UNIT_BRANCH | from_rr_i.instr.unit == UNIT_SYSTEM) begin
            stall_int = ~ready;
        end
    end
end



// Correct prediction
always_comb begin
    if(branch_to_scalar_wb.valid)begin
        if (from_rr_i.instr.instr_type == JAL)begin
            correct_branch_pred_o = 1'b1;
        end else   
        if (from_rr_i.instr.instr_type != BLT && from_rr_i.instr.instr_type != BLTU &&
            from_rr_i.instr.instr_type != BGE && from_rr_i.instr.instr_type != BGEU &&
            from_rr_i.instr.instr_type != BEQ && from_rr_i.instr.instr_type != BNE  &&
            from_rr_i.instr.instr_type != JALR) begin            
            correct_branch_pred_o = 1'b1; // Correct because Decode and Control Unit Already fixed the missprediciton
        end else begin
            if (from_rr_i.instr.bpred.is_branch) begin
                correct_branch_pred_o = (from_rr_i.instr.bpred.decision == branch_to_scalar_wb.branch_taken) &&
                                        (from_rr_i.instr.bpred.decision == PRED_NOT_TAKEN ||
                                        from_rr_i.instr.bpred.pred_addr == branch_to_scalar_wb.result_pc);
            end else begin
                correct_branch_pred_o = ~branch_to_scalar_wb.branch_taken;
            end
        end
    end else begin
        correct_branch_pred_o = 1'b1;
    end
    
end

// Branch predictor required signals
// Program counter at Execution Stage
assign exe_if_branch_pred_o.pc_execution = from_rr_i.instr.pc; 
// Final address generated by branch in Execution Stage
assign exe_if_branch_pred_o.branch_addr_result_exe = (branch_to_scalar_wb.branch_taken == PRED_TAKEN) ? branch_to_scalar_wb.result_pc : branch_to_scalar_wb.result;
// Target Address generated by branch in Execution Stage 
assign exe_if_branch_pred_o.branch_addr_target_exe = branch_to_scalar_wb.result_pc;
// Taken or not taken branch result in Execution Stage
assign exe_if_branch_pred_o.branch_taken_result_exe = branch_to_scalar_wb.branch_taken == PRED_TAKEN;   
// The instruction in the Execution Stage is a branch
assign exe_if_branch_pred_o.is_branch_exe = (from_rr_i.instr.instr_type == BLT  |
                                             from_rr_i.instr.instr_type == BLTU |
                                             from_rr_i.instr.instr_type == BGE  |
                                             from_rr_i.instr.instr_type == BGEU |
                                             from_rr_i.instr.instr_type == BEQ  |
                                             from_rr_i.instr.instr_type == BNE  |
                                             from_rr_i.instr.instr_type == JAL  |
                                             from_rr_i.instr.instr_type == JALR) &
                                             arith_instr.instr.valid;
                                             

// Data for the Control Unit
assign exe_cu_o.valid_1 = arith_to_scalar_wb_o.valid;
//assign exe_cu_o.valid_4 = mul_div_to_scalar_wb_o.valid;
assign exe_cu_o.change_pc_ena_1 = arith_to_scalar_wb_o.change_pc_ena;
assign exe_cu_o.is_branch = exe_if_branch_pred_o.is_branch_exe;
assign exe_cu_o.branch_taken = arith_to_scalar_wb_o.branch_taken;
assign exe_cu_o.stall = stall_int;


//-PMU 
assign pmu_is_branch_o       = from_rr_i.instr.bpred.is_branch && from_rr_i.instr.valid;
assign pmu_branch_taken_o    = from_rr_i.instr.bpred.is_branch && from_rr_i.instr.bpred.decision && 
                               from_rr_i.instr.valid;
                               
//assign pmu_miss_prediction_o = !correct_branch_pred_o;

assign pmu_exe_ready_o = ready;
assign pmu_struct_depend_stall_o = ready && stall_int;

endmodule
