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

//-----------------------------
// includes
//-----------------------------

`timescale 1 ns / 1 ns
`include "colors.vh"

import drac_pkg::*;

module tb_instruction_queue();

//-----------------------------
// Local parameters
//-----------------------------
    parameter VERBOSE         = 1;
    parameter CLK_PERIOD      = 2;
    parameter CLK_HALF_PERIOD = CLK_PERIOD / 2;

//-----------------------------
// Signals
//-----------------------------
    reg tb_clk_i;
    reg tb_rstn_i;
    logic tb_read_head_S_i[NUM_SCALAR_INSTR];
    logic tb_full_o;
    logic tb_flush_i;
    id_ir_stage_t tb_instruction_S_i[NUM_SCALAR_INSTR];
    id_ir_stage_t tb_instruction_S_o[NUM_SCALAR_INSTR];

//-----------------------------
// Module
//-----------------------------

    instruction_queue instruction_queue_inst(
        .clk_i(tb_clk_i),               
        .rstn_i(tb_rstn_i),             
        .instruction_S_i(tb_instruction_S_i),   
        .flush_i(tb_flush_i),
        .read_head_S_i(tb_read_head_S_i),
        .instruction_S_o(tb_instruction_S_o),
        .full_o(tb_full_o),
        .empty_o(tb_empty_o)
    );

//-----------------------------
// DUT
//-----------------------------


//***clk_gen***
// A single clock source is used in this design.
    initial tb_clk_i = 1;
    always #CLK_HALF_PERIOD tb_clk_i = !tb_clk_i;

    //***task automatic reset_dut***
    task automatic reset_dut;
        begin
            $display("*** Toggle reset.");
            tb_rstn_i <= 1'b0; 
            #CLK_PERIOD;
            tb_rstn_i <= 1'b1;
            #CLK_PERIOD;
            $display("Done");
        end
    endtask

//***task automatic init_sim***
    task automatic init_sim;
        begin
            $display("*** init_sim");
            tb_clk_i <='{default:1};
            tb_rstn_i<='{default:0};
            tb_read_head_S_i<='{default:0};
            tb_flush_i<='{default:0};
            tb_instruction_S_i<='{default:0};
            $display("Done");
        end
    endtask

//***task automatic init_dump***
//This task dumps the ALL signals of ALL levels of instance dut_example into the test.vcd file
//If you want a subset to modify the parameters of $dumpvars
    task automatic init_dump;
        begin
            $display("*** init_dump");
            $dumpfile("dump_file.vcd");
            $dumpvars(0,instruction_queue_inst);
        end
    endtask

    task automatic tick();
        begin
            //$display("*** tick");
            #CLK_PERIOD;
        end
    endtask


// Check reset of free list
    task automatic test_sim_1;
        output int tmp;
        begin
            tmp = 0;
            #CLK_PERIOD;
            assert(tb_full_o == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.head[0] == 0)  else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.tail[0] == 0)  else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.num[0] == 0) else begin tmp++; assert(1 == 0); end
    
            #CLK_PERIOD;

        end
    endtask


// Reads some free registers and then frees other 8 registers
// No checkpointing involved
    task automatic test_sim_2;
        output int tmp;
        begin
            tick();
            for(int i = 0; i < NUM_SCALAR_INSTR; i++) begin
                tb_instruction_S_i[i].instr = '{   
                    valid: 1'b1,                   // Valid instruction
                    pc: addrPC_t'(1),           // PC of the instruction
                    instr_type: instr_type_t'(1),       // Type of instruction
                    rd: reg_t'(1),              // Destination Register
                    rs1: reg_t'(1),              // Source register 1
                    default: '0
                };
            end    
            for(int i=0; i<16; i+=2) begin            // Reads 32 free registers

                tick();
                
                assert(instruction_queue_inst.head == 3'h0)  else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.tail == i) else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.num == i) else begin tmp++; assert(1 == 0); end
               
            end

            tick(); // Tries to read but is empty

            assert(tb_empty_o == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.head == 0) else begin tmp++; assert(1 == 0); end          
            assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.num == 16) else begin tmp++; assert(1 == 0); end
            tb_instruction_S_i = {default:0};
            tick(); 

            tb_read_head_S_i= {1,1};

            for(int i=0; i<16; i+=2) begin            // Reads 32 free registers

                tick();
                
                assert(instruction_queue_inst.head == i)  else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.num == 16-i) else begin tmp++; assert(1 == 0); end
               
            end

            tick();
            assert(tb_empty_o == 1) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.head == 0) else begin tmp++; assert(1 == 0); end          
            assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.num == 0) else begin tmp++; assert(1 == 0); end

             tb_read_head_S_i= {0,0};

        end
    endtask
    task automatic test_sim_3;
        output int tmp;
        begin
            tick();
            
            tb_instruction_S_i[0].instr = '{   
                valid: 1'b1,                   // Valid instruction
                pc: addrPC_t'(1),           // PC of the instruction
                instr_type: instr_type_t'(1),       // Type of instruction
                rd: reg_t'(1),              // Destination Register
                rs1: reg_t'(1),              // Source register 1
                default: '0
            };
              
            for(int i=0; i<16; i++) begin            // Reads 32 free registers

                tick();
                
                assert(instruction_queue_inst.head == 3'h0)  else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.tail == i) else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.num == i) else begin tmp++; assert(1 == 0); end
               
            end

            tick(); // Tries to read but is empty

            assert(tb_empty_o == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.head == 0) else begin tmp++; assert(1 == 0); end          
            assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.num == 16) else begin tmp++; assert(1 == 0); end
            tb_instruction_S_i = {default:0};
            tick(); 

            tb_read_head_S_i[0] = 1;

            for(int i=0; i<16; i++) begin            // Reads 32 free registers

                tick();
                
                assert(instruction_queue_inst.head == i)  else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
                assert(instruction_queue_inst.num == 16-i) else begin tmp++; assert(1 == 0); end
               
            end

            tick();
            assert(tb_empty_o == 1) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.head == 0) else begin tmp++; assert(1 == 0); end          
            assert(instruction_queue_inst.tail == 0) else begin tmp++; assert(1 == 0); end
            assert(instruction_queue_inst.num == 0) else begin tmp++; assert(1 == 0); end



        end
    endtask
//***task automatic test_sim***
    task automatic test_sim;
        begin
            int tmp;
            $display("*** test_sim");
            // check reset
            test_sim_1(tmp); 
            if (tmp >= 1) begin
                `START_RED_PRINT
                        $display("TEST 1 FAILED.");
                `END_COLOR_PRINT
            end else begin
                `START_GREEN_PRINT
                        $display("TEST 1 PASSED.");
                `END_COLOR_PRINT
            end
            // Check reading and writing to free list
            test_sim_2(tmp); 
            if (tmp >= 1) begin
                `START_RED_PRINT
                        $display("TEST 2 FAILED.");
                `END_COLOR_PRINT
            end else begin
                `START_GREEN_PRINT
                        $display("TEST 2 PASSED.");
                `END_COLOR_PRINT
            end

            test_sim_3(tmp); 
            if (tmp >= 1) begin
                `START_RED_PRINT
                        $display("TEST 3 FAILED.");
                `END_COLOR_PRINT
            end else begin
                `START_GREEN_PRINT
                        $display("TEST 3 PASSED.");
                `END_COLOR_PRINT
            end
        end
    endtask


//***init_sim***
//The tasks that compose my testbench are executed here, feel free to add more tasks.
    initial begin
        init_sim();
        init_dump();
        reset_dut();
        test_sim();
        $finish;
    end


endmodule
