// ============================================================
// MIPS_Testbench.sv
// DV-style Verification for MIPS_pipeline
//
// 架構（模擬 UVM 分層）：
//   load_prog   ← 類比 sequence：把指令載入 IMEM
//   drv_*       ← 類比 driver：控制 RST、HALT
//   mon_*       ← 類比 monitor：觀察 pipeline 狀態
//   mb_write/sb_check ← 類比 TLM fifo + scoreboard
//   ref_*       ← golden model：純軟體計算預期值
//   cov_*       ← 功能覆蓋率
// ============================================================
`timescale 1ns/1ps

// ============================================================
// Transaction 定義（類比 uvm_sequence_item）
// 欄位：{op_id[3:0], got[31:0], exp[31:0]}
// ============================================================
`define MBOX_DEPTH 64

// op_id
`define OP_ADD    4'd0
`define OP_ADDI   4'd1
`define OP_SUB    4'd2
`define OP_AND    4'd3
`define OP_OR     4'd4
`define OP_XOR    4'd5
`define OP_SLL    4'd6
`define OP_SRL    4'd7
`define OP_LUI    4'd8
`define OP_LW     4'd9
`define OP_SW     4'd10
`define OP_BEQ    4'd11
`define OP_BNE    4'd12
`define OP_J      4'd13
`define OP_JAL    4'd14
`define OP_JR     4'd15

`define OP_SLT    5'd16
`define OP_RBIT   5'd17
`define OP_REV    5'd18
`define OP_ADD8   5'd19
`define OP_SADD   5'd20
`define OP_SSUB   5'd21
`define OP_RESET  5'd22
`define OP_HALT   5'd23
`define OP_FWD    5'd24
`define OP_STALL  5'd25

module MIPS_Testbench;

  reg CLK;
  initial CLK = 0;
  always #5 CLK = ~CLK;

  // ---- DUT signal -------------------------------------------
  reg        RST, HALT;
  wire [31:0] reg1_out, reg2_out;

  // ---- DUT ------------------------------------------------
  MIPS_pipeline DUT (
    .CLK      (CLK),
    .RST      (RST),
    .HALT     (HALT),
    .reg1_out (reg1_out),
    .reg2_out (reg2_out)
  );

  // ---- cycle counter --------------------------------------
  integer cyc;
  initial cyc = 0;
  always @(posedge CLK) cyc = cyc + 1;

  // ---- DUT signals ---------
  reg [6:0]  PC_r;
  reg [31:0] if_id_i, id_ex_i;
  reg [4:0]  id_ex_rd_r;
  reg        fwd_ae, fwd_am, fwd_be, fwd_bm;
  reg        stall_r, flush_r;

  always @(*) begin
    PC_r       = DUT.PC;
    if_id_i    = DUT.if_id_instr;
    id_ex_i    = DUT.id_ex_instr;
    id_ex_rd_r = DUT.id_ex_rd;
    fwd_ae     = DUT.fwd_a_ex;
    fwd_am     = DUT.fwd_a_mem;
    fwd_be     = DUT.fwd_b_ex;
    fwd_bm     = DUT.fwd_b_mem;
    stall_r    = DUT.stall;
    flush_r    = DUT.flush;
  end

  // ============================================================
  // Mailbox（Monitor → Scoreboard，類比 uvm_tlm_fifo）
  // 每筆：{op_id[4:0], got[31:0], exp[31:0]}
  // ============================================================
  reg [68:0] mbox [`MBOX_DEPTH-1:0];
  reg [5:0]  mbox_wptr, mbox_rptr;
  integer    mbox_cnt;

  integer total_checks, total_pass, total_fail;

  initial begin
    mbox_wptr = 0; mbox_rptr = 0; mbox_cnt = 0;
    total_checks = 0; total_pass = 0; total_fail = 0;
  end

  // ============================================================
  // DRIVER tasks (用pc看執行結束否)
  // ============================================================

  task drv_wait;
    input integer n;
    integer k;
    begin for (k=0; k<n; k=k+1) @(posedge CLK); end
  endtask

  task drv_reset;
    begin
      RST = 1; HALT = 0;
      @(posedge CLK); @(posedge CLK);
      RST = 0;
      @(posedge CLK);
    end
  endtask

  // 等 PC 到 tgt，最多 tmo cycles
  task drv_wait_pc;
    input [6:0] tgt;
    input integer tmo;
    integer k;
    begin
      k = 0;
      while (DUT.PC !== tgt && k < tmo) begin
        @(posedge CLK); k = k + 1;
      end
      if (k >= tmo)
        $display("  [DRIVER][TIMEOUT] PC 未到 %0d (current %0d)", tgt, DUT.PC);
    end
  endtask

  // 載入程式到 IMEM，並 reset (instead of 讀.hex)
  // 最多支援 16 條指令（不夠可以直接在 task 外改 IMEM）
  task load_prog;
    input [31:0] i0,  i1,  i2,  i3;
    input [31:0] i4,  i5,  i6,  i7;
    input [31:0] i8,  i9,  i10, i11;
    input [31:0] i12, i13, i14, i15;
    integer k;
    begin
      // 先全填 NOP
      for (k=0; k<128; k=k+1) DUT.IMEM[k] = 32'h0;
      // 再載入指令
      DUT.IMEM[ 0] = i0;  DUT.IMEM[ 1] = i1;
      DUT.IMEM[ 2] = i2;  DUT.IMEM[ 3] = i3;
      DUT.IMEM[ 4] = i4;  DUT.IMEM[ 5] = i5;
      DUT.IMEM[ 6] = i6;  DUT.IMEM[ 7] = i7;
      DUT.IMEM[ 8] = i8;  DUT.IMEM[ 9] = i9;
      DUT.IMEM[10] = i10; DUT.IMEM[11] = i11;
      DUT.IMEM[12] = i12; DUT.IMEM[13] = i13;
      DUT.IMEM[14] = i14; DUT.IMEM[15] = i15;
      // Reset（讓 pipeline 從頭開始跑）
      drv_reset;
    end
  endtask

  // 等 pipeline drain：等 PC 超過最後一條有效指令，再等 5 cycles
  task drv_drain;
    input [6:0] last_instr_pc; // 最後一條指令的 pc（非 NOP）
    begin
      // 等 PC 跑到最後指令後的第一個 NOP
      drv_wait_pc(last_instr_pc + 7'd1, 60);
      // 再等 5 cycles 讓 pipeline drain 完
      drv_wait(5);
    end
  endtask

  // ============================================================
  // MONITOR tasks
  // ============================================================

  // 印一行 pipeline 狀態
  task mon_pp;
    begin
      $display("  cyc=%-3d PC=%02d | IF=%08h ID=%08h | fwdA=%b%b fwdB=%b%b stall=%b flush=%b | EX_rd=%02d EX_res=%08h",
        cyc, PC_r, if_id_i, id_ex_i,
        fwd_ae, fwd_am, fwd_be, fwd_bm,
        stall_r, flush_r,
        id_ex_rd_r, DUT.alu_result);
    end
  endtask

  // 觀察一段 cycles，有 hazard 事件就印出
  task mon_watch;
    input integer n;
    integer k;
    begin
      for (k=0; k<n; k=k+1) begin
        @(posedge CLK);
        if (stall_r || flush_r || fwd_ae || fwd_am || fwd_be || fwd_bm)
          mon_pp;
      end
    end
  endtask

  // ============================================================
  // SCOREBOARD tasks
  // ============================================================

  task mb_write;
    input [4:0]  op_id;
    input [31:0] got, exp;
    begin
      mbox[mbox_wptr] = {op_id, got, exp};
      mbox_wptr = mbox_wptr + 1;
      mbox_cnt  = mbox_cnt  + 1;
    end
  endtask

  task sb_check;
    reg [4:0]  op_id;
    reg [31:0] got, exp;
    reg [79:0] name;
    begin
      while (mbox_cnt == 0) @(posedge CLK);
      {op_id, got, exp} = mbox[mbox_rptr];
      mbox_rptr = mbox_rptr + 1;
      mbox_cnt  = mbox_cnt  - 1;

      case (op_id)
        `OP_ADD:   name = "ADD       ";
        `OP_ADDI:  name = "ADDI      ";
        `OP_SUB:   name = "SUB       ";
        `OP_AND:   name = "AND       ";
        `OP_OR:    name = "OR        ";
        `OP_XOR:   name = "XOR       ";
        `OP_SLL:   name = "SLL       ";
        `OP_SRL:   name = "SRL       ";
        `OP_LUI:   name = "LUI       ";
        `OP_LW:    name = "LW        ";
        `OP_SW:    name = "SW(DMEM)  ";
        `OP_BEQ:   name = "BEQ       ";
        `OP_BNE:   name = "BNE       ";
        `OP_J:     name = "J         ";
        `OP_JAL:   name = "JAL($31)  ";
        `OP_JR:    name = "JR        ";
        `OP_SLT:   name = "SLT       ";
        `OP_RBIT:  name = "RBIT      ";
        `OP_REV:   name = "REV       ";
        `OP_ADD8:  name = "ADD8      ";
        `OP_SADD:  name = "SADD      ";
        `OP_SSUB:  name = "SSUB      ";
        `OP_RESET: name = "RESET     ";
        `OP_HALT:  name = "HALT      ";
        `OP_FWD:   name = "FORWARDING";
        `OP_STALL: name = "LOAD-USE  ";
        default:   name = "UNKNOWN   ";
      endcase

      total_checks = total_checks + 1;
      if (got === exp) begin
        total_pass = total_pass + 1;
        $display("  [PASS] %-10s  got=0x%08h", name, got);
      end else begin
        total_fail = total_fail + 1;
        $display("  [FAIL] %-10s  got=0x%08h  exp=0x%08h  ←", name, got, exp);
      end
    end
  endtask

  task sb_report;
    begin
      $display("\n============================================================");
      $display("  SCOREBOARD REPORT");
      $display("  Total : %0d  |  PASS : %0d  |  FAIL : %0d",
               total_checks, total_pass, total_fail);
      if (total_fail == 0)
        $display("  *** ALL TESTS PASSED ***");
      else
        $display("  *** %0d TESTS FAILED ***", total_fail);
      $display("============================================================");
    end
  endtask

  // ============================================================
  // COVERAGE
  // ============================================================
  reg [25:0] cov_op_hit;   // bit[i] = op_id i 有被測到
  reg        cov_fwd_ex_hit, cov_fwd_mem_hit;
  reg        cov_stall_hit, cov_flush_hit;
  reg        cov_halt_hit,  cov_reset_hit;

  initial begin
    cov_op_hit    = 26'h0;
    cov_fwd_ex_hit  = 0; cov_fwd_mem_hit = 0;
    cov_stall_hit   = 0; cov_flush_hit   = 0;
    cov_halt_hit    = 0; cov_reset_hit   = 0;
  end

  task cov_sample;
    input [4:0] op_id;
    begin
      if (op_id < 26) cov_op_hit[op_id] = 1;
    end
  endtask

  // 自動採樣 hazard coverage（always block 裡）
  always @(posedge CLK) begin
    if (!RST) begin
      if (fwd_ae || fwd_be) cov_fwd_ex_hit  <= 1;
      if (fwd_am || fwd_bm) cov_fwd_mem_hit <= 1;
      if (stall_r)          cov_stall_hit   <= 1;
      if (flush_r)          cov_flush_hit   <= 1;
    end
  end

  task cov_report;
    integer i, hit;
    real pct;
    reg [79:0] op_names [0:25];
    begin
      op_names[0]  = "ADD       "; op_names[1]  = "ADDI      ";
      op_names[2]  = "SUB       "; op_names[3]  = "AND       ";
      op_names[4]  = "OR        "; op_names[5]  = "XOR       ";
      op_names[6]  = "SLL       "; op_names[7]  = "SRL       ";
      op_names[8]  = "LUI       "; op_names[9]  = "LW        ";
      op_names[10] = "SW        "; op_names[11] = "BEQ       ";
      op_names[12] = "BNE       "; op_names[13] = "J         ";
      op_names[14] = "JAL       "; op_names[15] = "JR        ";
      op_names[16] = "SLT       "; op_names[17] = "RBIT      ";
      op_names[18] = "REV       "; op_names[19] = "ADD8      ";
      op_names[20] = "SADD      "; op_names[21] = "SSUB      ";
      op_names[22] = "RESET     "; op_names[23] = "HALT      ";
      op_names[24] = "FORWARDING"; op_names[25] = "LOAD-USE  ";

      $display("\n============================================================");
      $display("  COVERAGE REPORT");

      hit = 0;
      for (i=0; i<22; i=i+1) if (cov_op_hit[i]) hit = hit + 1;
      pct = hit * 100.0 / 22.0;
      $display("  Instruction coverage : %0d/22 (%.1f%%)", hit, pct);
      for (i=0; i<22; i=i+1)
        $display("    %-10s : %s", op_names[i], cov_op_hit[i] ? "HIT" : "MISS");

      $display("  Hazard coverage:");
      $display("    EX/MEM→EX forwarding : %s", cov_fwd_ex_hit  ? "HIT" : "MISS");
      $display("    MEM/WB→EX forwarding : %s", cov_fwd_mem_hit ? "HIT" : "MISS");
      $display("    Load-use stall       : %s", cov_stall_hit   ? "HIT" : "MISS");
      $display("    Branch/Jump flush    : %s", cov_flush_hit   ? "HIT" : "MISS");
      $display("    HALT                 : %s", cov_halt_hit    ? "HIT" : "MISS");
      $display("    RESET                : %s", cov_reset_hit   ? "HIT" : "MISS");
      $display("============================================================");
    end
  endtask

  // ============================================================
  // REFERENCE MODEL
  // ============================================================
  task ref_alu;
    input [31:0] A, B;
    input [4:0]  op_id;  // 用 OP_* 定義
    input [4:0]  shamt;
    output [31:0] result;
    reg [32:0] tmp;
    integer i;
    begin
      case (op_id)
        `OP_ADD:  result = A + B;
        `OP_ADDI: result = A + B;
        `OP_SUB:  result = A - B;
        `OP_AND:  result = A & B;
        `OP_OR:   result = A | B;
        `OP_XOR:  result = A ^ B;
        `OP_SLL:  result = B << shamt;
        `OP_SRL:  result = B >> shamt;
        `OP_SLT:  result = (A < B) ? 32'd1 : 32'd0;
        `OP_LUI:  result = {B[15:0], 16'h0};
        `OP_RBIT: begin
          for (i=0; i<32; i=i+1) result[i] = B[31-i];
        end
        `OP_REV:  result = {B[7:0], B[15:8], B[23:16], B[31:24]};
        `OP_ADD8: result = {A[31:24]+B[31:24], A[23:16]+B[23:16],
                            A[15:8]+B[15:8],   A[7:0]+B[7:0]};
        `OP_SADD: begin
          tmp    = {1'b0,A} + {1'b0,B};
          result = tmp[32] ? 32'hFFFFFFFF : tmp[31:0];
        end
        `OP_SSUB: result = (A < B) ? 32'h0 : A - B;
        default:  result = 32'hDEAD_BEEF;
      endcase
    end
  endtask

  // ============================================================
  // INSTRUCTION ENCODING HELPERS
  // ============================================================
  // R-type: op=0 rs rt rd shamt funct
  function [31:0] enc_r;
    input [4:0] rs, rt, rd, shamt;
    input [5:0] funct;
    begin enc_r = {6'b0, rs, rt, rd, shamt, funct}; end
  endfunction

  // I-type: op rs rt imm16
  function [31:0] enc_i;
    input [5:0]  op;
    input [4:0]  rs, rt;
    input [15:0] imm;
    begin enc_i = {op, rs, rt, imm}; end
  endfunction

  // J-type: op target26
  function [31:0] enc_j;
    input [5:0]  op;
    input [25:0] target;
    begin enc_j = {op, target}; end
  endfunction

  // ============================================================
  // MAIN TEST SEQUENCE
  // ============================================================
  reg [31:0] exp_val;
  reg [31:0] ref_A, ref_B;

  initial begin
    RST = 1; HALT = 0;
    drv_wait(2);

    $display("============================================================");
    $display("  MIPS_pipeline DV-style Verification");
    $display("============================================================");
    
    // ==========================================================
    // PHASE 1: RESET
    // ==========================================================
    $display("\n===== PHASE 1: Reset =====");
    drv_reset;
    cov_reset_hit = 1;
    if (DUT.PC === 7'd0) begin
      $display("  [MONITOR] after RST : PC=0");
      mb_write(`OP_RESET, 32'd0, 32'd0);
    end else begin
      mb_write(`OP_RESET, {25'd0, DUT.PC}, 32'd0);
    end
    cov_sample(`OP_RESET);
    sb_check;

    // ==========================================================
    // PHASE 2: 基本指令測試
    // ==========================================================
    $display("\n===== PHASE 2: Basic Instructions =====");

    // ---- ADD: $2 = $3 + $4（$3=10, $4=7 → $2=17）----------
    $display("\n  [TEST] ADD $2=$3+$4 (10+7=17)");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd10),  // addi $3,$0,10
      enc_i(6'b001000, 5'd0, 5'd4, 16'd7),   // addi $4,$0,7
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b100000), // add $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    $display("  [MONITOR] Pipeline (include EX forwarding):");
    mon_watch(10);
    ref_alu(32'd10, 32'd7, `OP_ADD, 5'd0, exp_val);
    mb_write(`OP_ADD, DUT.REGS[2], exp_val);
    cov_sample(`OP_ADD);
    sb_check;

    // ---- SUB: $2 = $3 - $4（20-5=15）----------------------
    $display("\n  [TEST] SUB $2=$3-$4 (20-5=15)");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd20),  // addi $3,$0,20
      enc_i(6'b001000, 5'd0, 5'd4, 16'd5),   // addi $4,$0,5
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b100010), // sub $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'd20, 32'd5, `OP_SUB, 5'd0, exp_val);
    mb_write(`OP_SUB, DUT.REGS[2], exp_val);
    cov_sample(`OP_SUB);
    sb_check;

    // ---- ADDI: $2 = $0 + 42 = 42 --------------------------
    $display("\n  [TEST] ADDI $2=$0+42");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd2, 16'd42),  // addi $2,$0,42
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    ref_alu(32'd0, 32'd42, `OP_ADDI, 5'd0, exp_val);
    mb_write(`OP_ADDI, DUT.REGS[2], exp_val);
    cov_sample(`OP_ADDI);
    sb_check;

    // ---- AND -------------------------------------------
    $display("\n  [TEST] AND $2=$3&$4 (0xFFFF0000 & 0x0F0F0F0F = 0x0F0F0000)");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'hFFFF),  // lui $3,0xFFFF
      enc_i(6'b001111, 5'd0, 5'd4, 16'h0F0F),  // lui $4,0x0F0F
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b100100), // and $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'hFFFF0000, 32'h0F0F0000, `OP_AND, 5'd0, exp_val);
    mb_write(`OP_AND, DUT.REGS[2], exp_val);
    cov_sample(`OP_AND);
    sb_check;

    // ---- OR --------------------------------------------
    $display("\n  [TEST] OR $2=$3|$4 (0xF0F00000 | 0x0F0F0000 = 0xFFFF0000)");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'hF0F0),
      enc_i(6'b001111, 5'd0, 5'd4, 16'h0F0F),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b100101), // or $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'hF0F00000, 32'h0F0F0000, `OP_OR, 5'd0, exp_val);
    mb_write(`OP_OR, DUT.REGS[2], exp_val);
    cov_sample(`OP_OR);
    sb_check;

    // ---- XOR -------------------------------------------
    $display("\n  [TEST] XOR $2=$3^$4 (0xFFFF0000 ^ 0xFF000000 = 0x00FF0000)");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'hFFFF),
      enc_i(6'b001111, 5'd0, 5'd4, 16'hFF00),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b100110), // xor $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'hFFFF0000, 32'hFF000000, `OP_XOR, 5'd0, exp_val);
    mb_write(`OP_XOR, DUT.REGS[2], exp_val);
    cov_sample(`OP_XOR);
    sb_check;

    // ---- SLL: $2 = $3 << 4（1<<4=16）-------------------
    $display("\n  [TEST] SLL $2=$3<<4 (1<<4=16)");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd1),      // addi $3,$0,1
      enc_r(5'd0, 5'd3, 5'd2, 5'd4, 6'b000000), // sll $2,$3,4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    mon_watch(10);
    ref_alu(32'd0, 32'd1, `OP_SLL, 5'd4, exp_val);
    mb_write(`OP_SLL, DUT.REGS[2], exp_val);
    cov_sample(`OP_SLL);
    sb_check;

    // ---- SRL: $2 = $3 >> 4（0x00F00000>>4=0x000F0000）--
    $display("\n  [TEST] SRL $2=$3>>4 (0x00F00000>>4=0x000F0000)");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h00F0),   // lui $3,0x00F0
      enc_r(5'd0, 5'd3, 5'd2, 5'd4, 6'b000010), // srl $2,$3,4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    mon_watch(10);
    ref_alu(32'd0, 32'h00F00000, `OP_SRL, 5'd4, exp_val);
    mb_write(`OP_SRL, DUT.REGS[2], exp_val);
    cov_sample(`OP_SRL);
    sb_check;

    // ---- LUI -------------------------------------------
    $display("\n  [TEST] LUI $2,0xABCD → $2=0xABCD0000");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd2, 16'hABCD),  // lui $2,0xABCD
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd0);
    mb_write(`OP_LUI, DUT.REGS[2], 32'hABCD0000);
    cov_sample(`OP_LUI);
    sb_check;

    // ---- SLT -------------------------------------------
    $display("\n  [TEST] SLT $2=$3<$4? (5<10=1)");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd5),
      enc_i(6'b001000, 5'd0, 5'd4, 16'd10),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b101010), // slt $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    mb_write(`OP_SLT, DUT.REGS[2], 32'd1);
    cov_sample(`OP_SLT);
    sb_check;

    // ==========================================================
    // PHASE 3: 記憶體指令
    // ==========================================================
    $display("\n===== PHASE 3: Memory Instructions =====");

    // ---- LW / SW -------------------------------------------
    // sw $3, 0($0)   → DMEM[0] = 0xDEAD
    // lw $2, 0($0)   → $2 = 0xDEAD
    $display("\n  [TEST] SW+LW: store 0xDEAD → load → $2=0xDEAD");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'hDEAD),  // addi $3,$0,0xDEAD（負數，但夠示範）
      enc_i(6'b101011, 5'd0, 5'd3, 16'd0),      // sw $3,0($0)
      enc_i(6'b100011, 5'd0, 5'd2, 16'd0),      // lw $2,0($0)
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    $display("  [MONITOR] Pipeline (include stall/forwarding):");
    mon_watch(20);
    mb_write(`OP_SW, DUT.DMEM[0], 32'hFFFFDEAD); // addi sign-extends
    mb_write(`OP_LW, DUT.REGS[2], 32'hFFFFDEAD);
    cov_sample(`OP_SW);
    cov_sample(`OP_LW);
    sb_check; sb_check;
  
    // ==========================================================
    // PHASE 4: 分支與跳轉
    // ==========================================================
    $display("\n===== PHASE 4: Branch / Jump =====");

    // ---- BEQ taken: $3==$3 → 跳到 addr 5（NOP 區）---------
    // beq $3,$3,3  → PC+1+3 = addr 5
    $display("\n  [TEST] BEQ taken: skip 2 instr, $2 should stay 0");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd99),    // addi $3,$0,99
      enc_i(6'b000100, 5'd3, 5'd3, 16'd2),     // beq $3,$3,2 → skip to addr 4
      enc_i(6'b001000, 5'd0, 5'd2, 16'hBAAF),  // addi $2,$0,0xBEEF (should be skipped)
      enc_i(6'b001000, 5'd0, 5'd2, 16'hBEEF),  // addi $2,$0,0xBEEF (should be skipped)
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd3);
    $display("  [MONITOR] flush observation:");
    mon_watch(20);
    // $2 應保持 0（被 skip 的指令沒執行）
    mb_write(`OP_BEQ, DUT.REGS[2], 32'h0);
    cov_sample(`OP_BEQ);
    sb_check;

    // ---- BNE not taken: $3 != $4 → 不跳，$2 被寫入 --------
    $display("\n  [TEST] BNE not taken: $2 should be written");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd1),    // addi $3,$0,1
      enc_i(6'b001000, 5'd0, 5'd4, 16'd2),    // addi $4,$0,2
      enc_i(6'b000101, 5'd3, 5'd4, 16'd5),    // bne $3,$4,5 → taken！跳走
      enc_i(6'b001000, 5'd0, 5'd2, 16'hAAAA), // addi $2,$0,0xAAAA (skipped)
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd3);
    mon_watch(20);
    // bne 會 taken（$3≠$4），$2 保持 0
    mb_write(`OP_BNE, DUT.REGS[2], 32'h0);
    cov_sample(`OP_BNE);
    sb_check;

    // ---- J: 跳到 addr 3，跳過中間的 addi ------------------
    $display("\n  [TEST] J: jump to addr 3, skip addr 1~2");
    load_prog(
      enc_j(6'b000010, 26'd3),                // j 3
      enc_i(6'b001000, 5'd0, 5'd2, 16'hDEAD), // addi (skipped)
      enc_i(6'b001000, 5'd0, 5'd2, 16'hDEAD), // addi (skipped)
      enc_i(6'b001000, 5'd0, 5'd2, 16'h1234), // addi $2,$0,0x1234
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd3);
    mon_watch(15);
    mb_write(`OP_J, DUT.REGS[2], 32'h00001234);
    cov_sample(`OP_J);
    sb_check;

    // ---- JAL + JR ------------------------------------------
    // jal 5        → $31 = 1，PC = 5
    // nop nop nop (addr 1~4，不會執行)
    // addi $2,$0,0x5A5A  (addr 5，subroutine)
    // jr $31       → PC = 1
    $display("\n  [TEST] JAL+JR: $31=1, subroutine writes $2=0x5A5A, jr returns");
    load_prog(
      enc_j(6'b000011, 26'd5),                 // jal 5   (addr 0)
      32'h0,                                   // nop     (addr 1, return point)
      32'h0, 32'h0, 32'h0,
      enc_i(6'b001000, 5'd0, 5'd2, 16'h5A5A), // addi $2,$0,0x5A5A (addr 5)
      enc_r(5'd31, 5'd0, 5'd0, 5'd0, 6'b001000), // jr $31 (addr 6)
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd6);
    mon_watch(25);
    mb_write(`OP_JAL, DUT.REGS[31], 32'd1);  // $31 應為 1（JAL 後的 PC）
    mb_write(`OP_JR,  DUT.REGS[2],  32'h00005A5A);
    cov_sample(`OP_JAL);
    cov_sample(`OP_JR);
    sb_check; sb_check;

    // ==========================================================
    // PHASE 5: Hazard 專項測試
    // ==========================================================
    $display("\n===== PHASE 5: Hazard Tests =====");

    // ---- EX/MEM→EX Forwarding ------------------------------
    // add $3,$0,$0 → $3=0（讓 fwd 有意義）
    // addi $3,$0,7 → $3=7（這條寫 $3）
    // add  $2,$3,$0 → $2=$3（需要 EX forwarding）
    $display("\n  [TEST] EX→EX Forwarding: addi $3,7 then add $2=$3");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd7),      // addi $3,$0,7
      enc_r(5'd3, 5'd0, 5'd2, 5'd0, 6'b100000), // add $2,$3,$0 ← fwd from EX
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    $display("  [MONITOR] should see fwdA=10 (EX forwarding):");
    mon_watch(12);
    mb_write(`OP_FWD, DUT.REGS[2], 32'd7);
    cov_sample(`OP_FWD);
    sb_check;

    // ---- MEM/WB→EX Forwarding ------------------------------
    $display("\n  [TEST] MEM→EX Forwarding: addi $3,15 | nop | add $2=$3");
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd3, 16'd15),     // addi $3,$0,15
      32'h0,                                    // nop (距離 2 cycles)
      enc_r(5'd3, 5'd0, 5'd2, 5'd0, 6'b100000), // add $2,$3,$0 ← fwd from MEM/WB
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    $display("  [MONITOR] should see fwdA=01 (MEM forwarding):");
    mon_watch(12);
    mb_write(`OP_FWD, DUT.REGS[2], 32'd15);
    cov_sample(`OP_FWD);
    sb_check;

    // ---- Load-Use Stall ------------------------------------
    // lw $3, 0($0)  → $3 = DMEM[0]
    // add $2,$3,$0  → 緊接使用 $3，必須 stall 1 cycle
    $display("\n  [TEST] Load-Use Stall: lw $3 then immediately use $3");
    // 先設 DMEM[0] = 0x12345678
    DUT.DMEM[0] = 32'h12345678;
    load_prog(
      enc_i(6'b100011, 5'd0, 5'd3, 16'd0),      // lw $3,0($0)
      enc_r(5'd3, 5'd0, 5'd2, 5'd0, 6'b100000), // add $2,$3,$0 ← load-use!
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    $display("  [MONITOR] should see stall=1:");
    mon_watch(15);
    mb_write(`OP_STALL, DUT.REGS[2], 32'h12345678);
    cov_sample(`OP_STALL);
    sb_check;

    // ==========================================================
    // PHASE 6: 新增指令（RBIT/REV/ADD8/SADD/SSUB）
    // ==========================================================
    $display("\n===== PHASE 6: Extended Instructions =====");

    // ---- RBIT ----------------------------------------------
    $display("\n  [TEST] RBIT $2,$3: $3=0x7FFF0000 → 0x0000FFFE");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h7FFF),   // lui $3,0x7FFF
      enc_r(5'd0, 5'd3, 5'd2, 5'd0, 6'b101111), // rbit $2,$3
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    mon_watch(10);
    ref_alu(32'd0, 32'h7FFF0000, `OP_RBIT, 5'd0, exp_val);
    mb_write(`OP_RBIT, DUT.REGS[2], exp_val);
    cov_sample(`OP_RBIT);
    sb_check;

    // ---- REV -----------------------------------------------
    $display("\n  [TEST] REV $2,$3: $3=0x70000000 → 0x00000070");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h7000),   // lui $3,0x7000
      enc_r(5'd0, 5'd3, 5'd2, 5'd0, 6'b110000), // rev $2,$3
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd1);
    mon_watch(10);
    ref_alu(32'd0, 32'h70000000, `OP_REV, 5'd0, exp_val);
    mb_write(`OP_REV, DUT.REGS[2], exp_val);
    cov_sample(`OP_REV);
    sb_check;

    // ---- ADD8 ----------------------------------------------
    $display("\n  [TEST] ADD8 $2=$3+$4: 0x70000000+0x7FFF0000=0xEFFF0000");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h7000),   // lui $3,0x7000
      enc_i(6'b001111, 5'd0, 5'd4, 16'h7FFF),   // lui $4,0x7FFF
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b101101), // add8 $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'h70000000, 32'h7FFF0000, `OP_ADD8, 5'd0, exp_val);
    mb_write(`OP_ADD8, DUT.REGS[2], exp_val);
    cov_sample(`OP_ADD8);
    sb_check;

    // ---- SADD（不飽和）-------------------------------------
    $display("\n  [TEST] SADD $2=$3+$4: 0x7FFF0000+0x7FFF0000=0xFFFE0000 (no sat)");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h7FFF),
      enc_i(6'b001111, 5'd0, 5'd4, 16'h7FFF),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b110001), // sadd $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'h7FFF0000, 32'h7FFF0000, `OP_SADD, 5'd0, exp_val);
    mb_write(`OP_SADD, DUT.REGS[2], exp_val);
    cov_sample(`OP_SADD);
    sb_check;

    // ---- SADD（飽和）---------------------------------------
    $display("\n  [TEST] SADD saturate: 0xFFFF0000+0xFFFF0000→0xFFFFFFFF");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'hFFFF),
      enc_i(6'b001111, 5'd0, 5'd4, 16'hFFFF),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b110001),
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'hFFFF0000, 32'hFFFF0000, `OP_SADD, 5'd0, exp_val);
    mb_write(`OP_SADD, DUT.REGS[2], exp_val);
    sb_check;

    // ---- SSUB（飽和）---------------------------------------
    $display("\n  [TEST] SSUB saturate: 0x70000000-0x7FFF0000→0x0");
    load_prog(
      enc_i(6'b001111, 5'd0, 5'd3, 16'h7000),
      enc_i(6'b001111, 5'd0, 5'd4, 16'h7FFF),
      enc_r(5'd3, 5'd4, 5'd2, 5'd0, 6'b110010), // ssub $2,$3,$4
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_drain(7'd2);
    mon_watch(10);
    ref_alu(32'h70000000, 32'h7FFF0000, `OP_SSUB, 5'd0, exp_val);
    mb_write(`OP_SSUB, DUT.REGS[2], exp_val);
    cov_sample(`OP_SSUB);
    sb_check;

    // ==========================================================
    // PHASE 7: HALT
    // ==========================================================
    $display("\n===== PHASE 7: HALT =====");
    // 跑一個無窮迴圈程式，然後 HALT
    load_prog(
      enc_i(6'b001000, 5'd0, 5'd2, 16'd1),  // addi $2,$0,1
      enc_j(6'b000010, 26'd0),              // j 0（無窮迴圈）
      32'h0, 32'h0, 32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0
    );
    drv_wait(20);  // 讓它跑一陣子
    $display("  before HALT: PC=%0d  $2=0x%08h", DUT.PC, DUT.REGS[2]);
    HALT = 1;
    drv_wait(5);
    begin : halt_chk
      reg [6:0] fpc;
      reg [31:0] f2;
      integer hk;
      reg ok;
      fpc = DUT.PC; f2 = DUT.REGS[2]; ok = 1;
      $display("  freeze: PC=%0d", fpc);
      for (hk=0; hk<10; hk=hk+1) begin
        @(posedge CLK);
        if (DUT.PC !== fpc) begin ok=0;
          $display("  [FAIL] during HALT PC: %0d becomes %0d", fpc, DUT.PC);
        end
      end
      if (ok) $display("  [PASS] HALT 10 cycles PC frozen");
      mb_write(`OP_HALT, {31'd0, ok}, 32'd1);
      cov_halt_hit = 1;
    end
    cov_sample(`OP_HALT);
    sb_check;
    HALT = 0;
    $display("  release HALT: PC=%0d", DUT.PC);
    drv_wait(5);

    // ==========================================================
    // 報告
    // ==========================================================
    sb_report;
    cov_report;

    $finish;
  end

  // ---- 持續監控 reg2_out 變化 ------------------------------
  always @(reg2_out) begin
    if (!RST)
      $display("  [MON] reg2_out=0x%08h  cyc=%0d  PC=%0d",
               reg2_out, cyc, DUT.PC);
  end

endmodule