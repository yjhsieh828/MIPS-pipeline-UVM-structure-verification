`define OP if_id_instr[31:26] //R or op(I/J)
`define RS if_id_instr[25:21]
`define RT if_id_instr[20:16]
`define RD if_id_instr[15:11]
`define SHAMT if_id_instr[10:6]
`define FUNCT if_id_instr[5:0]
`define IMM if_id_instr[15:0]

module MIPS_pipeline (
  input CLK,
  input RST,
  input HALT,
  output [31:0] reg1_out,
  output [31:0] reg2_out
);

  parameter R = 2'd0;//special instructions (opcode == 000000), values of F code (bits 5-0):
  parameter f_add =   6'b100000;
  parameter f_sub =   6'b100010;
  parameter f_xor =  6'b100110;
  parameter f_and =  6'b100100;
  parameter f_or =   6'b100101;
  parameter f_slt =   6'b101010;
  parameter f_srl =   6'b000010;
  parameter f_sll =   6'b000000;
  parameter f_jr =    6'b001000;//IF->ID, and finish
  parameter f_rbit =  6'b101111;
  parameter f_rev =   6'b110000;
  parameter f_add8 =  6'b101101;
  parameter f_sadd =  6'b110001;
  parameter f_ssub =  6'b110010;
  //nop (goes to R-type)
  parameter f_nop =   6'b000000;

  parameter I = 2'd1;//non-special instructions, values of opcodes:
  parameter op_addi =  6'b001000;
  parameter op_andi =  6'b001100;
  parameter op_ori =   6'b001101;
  parameter op_lw =    6'b100011;
  parameter op_sw =    6'b101011;
  parameter op_beq =   6'b000100; //not use alu src, and use imm for branch target
  parameter op_bne =   6'b000101; //not use alu src, and use imm for branch target

  parameter J = 2'd2;
  parameter op_j =     6'b000010; //IF->ID, and finish
  parameter op_jal =   6'b000011; //IF->ID->MEM write to $31
  parameter op_lui =   6'b001111;
 
  //NOP instr (sll $0, $0, 0)
  parameter NOP_instr = 32'b0;

  // =================================================================
  // memory
  // (seperate IMEM & DMEM for structural hazard)
  // =================================================================

  reg [31:0] IMEM [0:127];
  initial begin
    integer ii;
      for (ii=0; ii<128; ii=ii+1) IMEM[ii] = 32'b0;
    //$readmemh("code.hex", IMEM, 0, 127);
  end

  reg [31:0] DMEM [0:127];
  initial begin
    integer jj;
      for (jj=0; jj<128; jj=jj+1) DMEM[jj] = 32'b0;
  end

  // =================================================================
  // register
  // =================================================================

  reg [31:0] REGS [0:31];
  //initial begin
    integer ri;
    always@(*) if(RST)for (ri=0; ri<32; ri=ri+1) REGS[ri] = 32'b0;
  //end
  assign reg1_out = REGS[1];
  assign reg2_out = REGS[2];

  // =================================================================
  // pc
  // =================================================================
  reg [6:0] PC;
  // =================================================================
  // pipeline registers
  // =================================================================
  // ----- IF/ID -----
  reg [31:0] if_id_instr;
  reg [6:0] if_id_pc; //PC+1 for jal
  // ----- ID/EX -----
  reg [31:0] id_ex_instr;
  reg [6:0] id_ex_pc; //PC+1 for jal
  reg [31:0] id_ex_rs_data;
  reg [31:0] id_ex_rt_data;
  reg [31:0] id_ex_imm_ext;
    //forwarding
  reg [4:0] id_ex_rs;
  reg [4:0] id_ex_rt;
  reg [4:0] id_ex_rd;//write (R: rd, I: rt)
    //control signals
  reg [5:0] id_ex_op;
  reg [5:0] id_ex_funct;
  reg [4:0] id_ex_shamt;
  reg id_ex_reg_write;
  reg id_ex_mem_read;
  reg id_ex_mem_write;
  reg id_ex_mem_to_reg;
  reg id_ex_alu_src; //0: reg, 1: imm
  reg id_ex_branch;
  reg id_ex_jr;
  reg id_ex_jal;
  // ----- EX/MEM -----
  reg [31:0] ex_mem_alu_result;
  reg [31:0] ex_mem_rt_data; //for sw
  reg [4:0] ex_mem_rd;
    //control signals
  reg [5:0] ex_mem_op;
  reg ex_mem_reg_write;
  reg ex_mem_mem_read;
  reg ex_mem_mem_write;
  reg ex_mem_mem_to_reg;
  // ----- MEM/WB -----
  reg [31:0] mem_wb_alu_result;
  reg [31:0] mem_wb_mem_data;
  reg [4:0] mem_wb_rd;
    //control signals
  reg mem_wb_reg_write;
  reg mem_wb_mem_to_reg;

  // =================================================================
  // ID stage (decoder)
  // (combinational logic: if_id_instr -> control signals, imm_ext, reg data)
  // =================================================================
  wire [5:0] dec_op = `OP;
  wire [4:0] dec_rs = `RS;
  wire [4:0] dec_rt = `RT;
  wire [4:0] dec_rd = `RD;
  wire [4:0] dec_shamt = `SHAMT;
  wire [5:0] dec_funct = `FUNCT;
  wire [15:0] dec_imm = `IMM;
    wire [31:0] dec_imm_ext = {{16{dec_imm[15]}}, dec_imm}; //sign-extend
  //rd
  wire [4:0] dec_dst = (dec_op == op_jal) ? 5'd31 : (dec_op == R) ? dec_rd : dec_rt;
  //control signals
  wire dec_reg_write = (dec_op == R && dec_funct != f_jr) ||
                        dec_op == op_addi || dec_op == op_andi||
                        dec_op == op_ori || dec_op == op_lw ||
                        dec_op == op_lui || dec_op == op_jal;
  wire dec_mem_read = (dec_op == op_lw);
  wire dec_mem_write = (dec_op == op_sw);
  wire dec_mem_to_reg = (dec_op == op_lw);
  wire dec_alu_src = (dec_op != R && dec_op != op_beq && dec_op != op_bne);
  wire dec_branch = (dec_op == op_beq || dec_op == op_bne);
  wire dec_jr = (dec_op == R && dec_funct == f_jr);
  wire dec_jal = (dec_op == op_jal);
  //reg data
  wire [31:0] dec_rs_data = REGS[dec_rs];
  wire [31:0] dec_rt_data = REGS[dec_rt];
  // =================================================================
  // hazard detection
  // (load-use hazard: ID see rs/rt == EX lw) (solved by stall)
  // =================================================================
  wire load_use_hazard = id_ex_mem_read &&
                       ((id_ex_rd == dec_rs && dec_rs != 5'b0) ||
                        (id_ex_rd == dec_rt && dec_rt != 5'b0));
  // =================================================================
  // stall
  // =================================================================
  wire stall = load_use_hazard;
  // =================================================================
  // halt
  // =================================================================
  reg halt_r;
  wire pipe_empty;
  wire pc_stall = stall || (HALT && halt_r);

  // =================================================================
  // forwarding rs/rt data
  // (read-after-write hazard: EX need rs/rt but not yet WB)
  // =================================================================
  //forward alu_in_A (rs)
  wire fwd_a_ex = ex_mem_reg_write && (ex_mem_rd != 5'b0) &&
                 (ex_mem_rd == id_ex_rs);
  wire fwd_a_mem = mem_wb_reg_write && (mem_wb_rd != 5'b0) &&
                  (mem_wb_rd == id_ex_rs) &&
                   ~fwd_a_ex;
  wire [31:0] fwd_rs_data = fwd_a_ex ? ex_mem_alu_result :
                            fwd_a_mem ? (mem_wb_mem_to_reg ? mem_wb_mem_data : mem_wb_alu_result) :
                            id_ex_rs_data; //from ex: alu result, mem: alu result or mem data (lw), else from ID
  //forward alu_in_B (rt)
  wire fwd_b_ex = ex_mem_reg_write && (ex_mem_rd != 5'b0) &&
                 (ex_mem_rd == id_ex_rt);
  wire fwd_b_mem = mem_wb_reg_write && (mem_wb_rd != 5'b0) &&
                  (mem_wb_rd == id_ex_rt) &&
                   ~fwd_b_ex;
  wire [31:0] fwd_rt_data = fwd_b_ex ? ex_mem_alu_result :
                            fwd_b_mem ? (mem_wb_mem_to_reg ? mem_wb_mem_data : mem_wb_alu_result) :
                            id_ex_rt_data; //from ex: alu result, mem: alu result or mem data (lw), else from ID
  //ALU input check
  wire [31:0] alu_in_A = fwd_rs_data;
  wire [31:0] alu_in_B = id_ex_alu_src ? id_ex_imm_ext : fwd_rt_data;//for alu src, check imm first, then rt data(with forwarding)

  // =================================================================
  // EX stage (alu)
  // =================================================================
  reg [31:0] alu_result;
  reg [32:0] sadd_tmp;

  always@(*) begin
    alu_result  = 32'b0;
    sadd_tmp = 33'b0;
    if(id_ex_op == R) begin
      case(id_ex_funct)
        f_add: alu_result = alu_in_A + alu_in_B;
        f_sub: alu_result = alu_in_A - alu_in_B;
        f_and: alu_result = alu_in_A & alu_in_B;
        f_or: alu_result = alu_in_A | alu_in_B;
        f_xor: alu_result = alu_in_A ^ alu_in_B;
        f_slt: alu_result = (alu_in_A < alu_in_B)? 32'd1 : 32'd0;
        f_srl: alu_result = alu_in_B >> id_ex_shamt;
        f_sll: alu_result = alu_in_B << id_ex_shamt;
        f_rbit: begin //for(i=0;i<32;i++){rs[i]=rt[31-i]}  
          alu_result[0] = alu_in_B[31];  alu_result[1] = alu_in_B[30];  alu_result[2] = alu_in_B[29];  alu_result[3] = alu_in_B[28];
          alu_result[4] = alu_in_B[27];  alu_result[5] = alu_in_B[26];  alu_result[6] = alu_in_B[25];  alu_result[7] = alu_in_B[24];
          alu_result[8] = alu_in_B[23];  alu_result[9] = alu_in_B[22];  alu_result[10] = alu_in_B[21]; alu_result[11] = alu_in_B[20];
          alu_result[12] = alu_in_B[19]; alu_result[13] = alu_in_B[18]; alu_result[14] = alu_in_B[17]; alu_result[15] = alu_in_B[16];
          alu_result[16] = alu_in_B[15]; alu_result[17] = alu_in_B[14]; alu_result[18] = alu_in_B[13]; alu_result[19] = alu_in_B[12];
          alu_result[20] = alu_in_B[11]; alu_result[21] = alu_in_B[10]; alu_result[22] = alu_in_B[9];  alu_result[23] = alu_in_B[8];
          alu_result[24] = alu_in_B[7];  alu_result[25] = alu_in_B[6];  alu_result[26] = alu_in_B[5];  alu_result[27] = alu_in_B[4];
          alu_result[28] = alu_in_B[3];  alu_result[29] = alu_in_B[2];  alu_result[30] = alu_in_B[1];  alu_result[31] = alu_in_B[0];            
        end
        f_rev: alu_result = {alu_in_B [7:0], alu_in_B[15:8], alu_in_B [23:16], alu_in_B [31:24]};
        //Reverse the bytes in a word. alu_result:rs, alu_in_A:rt
        f_add8: alu_result = {alu_in_A[31:24] + alu_in_B[31:24],
                            alu_in_A[23:16] + alu_in_B[23:16],
                            alu_in_A[15:8] + alu_in_B[15:8],
                            alu_in_A[7:0] + alu_in_B[7:0]}; //byte-wise addition. rd[]=rt[]+rs[]
        f_sadd: begin
          sadd_tmp = {1'b0, alu_in_A} + {1'b0, alu_in_B};
          alu_result = sadd_tmp[32] ? 32'hFFFFFFFF : sadd_tmp[31:0];
        end
        f_ssub: begin
          alu_result = (alu_in_A < alu_in_B) ? 32'h00000000 : (alu_in_A - alu_in_B);
        end
        default: alu_result = 32'b0;
      endcase
    end//end R-type
    else begin //I-type
      case(id_ex_op)
        op_addi: alu_result = alu_in_A + alu_in_B;
        op_andi: alu_result = alu_in_A & alu_in_B;
        op_ori: alu_result = alu_in_A | alu_in_B;
        op_lui:alu_result = {id_ex_imm_ext[15:0], 16'h0};
        op_lw, op_sw: alu_result = alu_in_A + alu_in_B; //calculate mem address
        op_beq, op_bne: alu_result = (alu_in_A == alu_in_B) ? 32'b1 : 32'b0; //for branch compare
        op_jal: alu_result = {25'h0, id_ex_pc}; //for jal, pass PC+1 to EX stage, and write to $31 in WB stage
        default: alu_result = 32'b0;
      endcase
    end//end I-type
  end
  // =================================================================
  // EX stage (branch select)
  // =================================================================
  wire branch_taken = id_ex_branch && ((id_ex_op == op_beq && alu_result == 32'b1) ||
                                       (id_ex_op == op_bne && alu_result == 32'b0));
  wire [6:0] branch_target = id_ex_pc + id_ex_imm_ext[6:0]; //PC+1+imm, and imm already sign-extended in ID stage
  wire [6:0] jump_target = id_ex_jr?  fwd_rs_data[6:0]
                         : id_ex_instr[6:0]; //j & jal
    // flush
  wire flush = branch_taken || id_ex_jr || id_ex_jal || (id_ex_op == op_j);
  wire [6:0] next_pc_flush = branch_taken ? branch_target : jump_target;

  // =================================================================
  // sequential cpu logic
  // (update pipeline registers, pc)
  // =================================================================
  always@(posedge CLK) begin
    if (RST) begin
      PC <= 7'b0;
      //flush all pipeline registers
      if_id_instr <= NOP_instr;
      if_id_pc <= 7'b0;
      id_ex_instr <= NOP_instr;
      id_ex_pc <= 7'b0;
      id_ex_rs_data <= 32'b0;
      id_ex_rt_data <= 32'b0;
      id_ex_imm_ext <= 32'b0;
      id_ex_rs <= 5'b0;
      id_ex_rt <= 5'b0;
      id_ex_rd <= 5'b0;
      id_ex_op <= 6'b0;
      id_ex_funct <= 6'b0;
      id_ex_shamt <= 5'b0;
      id_ex_reg_write <= 1'b0;
      id_ex_mem_read <= 1'b0;
      id_ex_mem_write <= 1'b0;

      id_ex_mem_to_reg <= 1'b0;
      id_ex_alu_src <= 1'b0;
      id_ex_branch <= 1'b0;
      id_ex_jr <= 1'b0;
      id_ex_jal <= 1'b0;

      ex_mem_alu_result <= 32'b0;
      ex_mem_rt_data <= 32'b0;
      ex_mem_rd <= 5'b0;
      ex_mem_op <= 6'b0;
      ex_mem_reg_write <= 1'b0;
      ex_mem_mem_read <= 1'b0;
      ex_mem_mem_write <= 1'b0;
      ex_mem_mem_to_reg <= 1'b0;

      mem_wb_alu_result <= 32'b0;
      mem_wb_mem_data <= 32'b0;
      mem_wb_rd <= 5'b0;      
      mem_wb_reg_write <= 1'b0;      
      mem_wb_mem_to_reg <= 1'b0;

      halt_r <= 1'b0;      
    end
    else begin
      halt_r <= HALT;
      // =================================
      // WB stage
      // (write from MEM to REGS)
      // =================================
      if (mem_wb_reg_write && mem_wb_rd != 5'b0) begin
        REGS[mem_wb_rd] <= mem_wb_mem_to_reg ? mem_wb_mem_data : mem_wb_alu_result; //lw: mem data, else: alu result
      end
      // =================================
      // MEM/WB pipeline register
      // =================================
      mem_wb_rd <= ex_mem_rd;
      mem_wb_reg_write <= ex_mem_reg_write;
      mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
      mem_wb_alu_result <= ex_mem_alu_result;
      // =================================
      // MEM stage
      // =================================
      if(ex_mem_mem_read)
        mem_wb_mem_data <= DMEM[ex_mem_alu_result[6:0]];
      else mem_wb_mem_data <= 32'b0; //default value for non-lw instructions
      if(ex_mem_mem_write)
        DMEM[ex_mem_alu_result[6:0]] <= ex_mem_rt_data;
      // =================================
      // EX/MEM pipeline register
      // =================================
      ex_mem_op <= id_ex_op;
      ex_mem_alu_result <= alu_result;
      ex_mem_rt_data <= fwd_rt_data; //for sw
      ex_mem_rd <= id_ex_rd;
      ex_mem_reg_write <= id_ex_reg_write;
      ex_mem_mem_read <= id_ex_mem_read;
      ex_mem_mem_write <= id_ex_mem_write;
      ex_mem_mem_to_reg <= id_ex_mem_to_reg;
      // =================================
      // ID/EX pipeline register
      // =================================
      if(pc_stall || flush) begin
        id_ex_instr <= NOP_instr;
        id_ex_pc <= 7'b0;
        id_ex_rs_data <= 32'b0;
        id_ex_rt_data <= 32'b0;
        id_ex_imm_ext <= 32'b0;
        id_ex_rs <= 5'b0;
        id_ex_rt <= 5'b0;
        id_ex_rd <= 5'b0;
        id_ex_op <= 6'b0;
        id_ex_funct <= 6'b0;
        id_ex_shamt <= 5'b0;
        id_ex_reg_write <= 1'b0;
        id_ex_mem_read <= 1'b0;
        id_ex_mem_write <= 1'b0;
        id_ex_mem_to_reg <= 1'b0;
        id_ex_alu_src <= 1'b0;
        id_ex_branch <= 1'b0;
        id_ex_jr <= 1'b0;
        id_ex_jal <= 1'b0;
      end else begin //normal proceed
        id_ex_instr <= if_id_instr;
        id_ex_pc <= if_id_pc;
        id_ex_rs_data <= dec_rs_data;
        id_ex_rt_data <= dec_rt_data;
        id_ex_imm_ext <= dec_imm_ext;
        id_ex_rs <= dec_rs;
        id_ex_rt <= dec_rt;
        id_ex_rd <= dec_dst;
        id_ex_op <= dec_op;
        id_ex_funct <= dec_funct;
        id_ex_shamt <= dec_shamt;
        id_ex_reg_write <= dec_reg_write;
        id_ex_mem_read <= dec_mem_read;
        id_ex_mem_write <= dec_mem_write;
        id_ex_mem_to_reg <= dec_mem_to_reg;
        id_ex_alu_src <= dec_alu_src;
        id_ex_branch <= dec_branch;
        id_ex_jr <= dec_jr;
        id_ex_jal <= dec_jal;
      end
      // =================================
      // IF/ID pipeline register
      // =================================
      if(flush) begin
        if_id_instr <= NOP_instr;
        if_id_pc <= 7'b0;
      end else if (!pc_stall) begin //normal fetch
        if_id_instr <= IMEM[PC];
        if_id_pc <= PC + 7'b1; //PC+1 for jal
      end else begin //stall, keep the same instruction in IF/ID
        if_id_instr <= if_id_instr;
        if_id_pc <= if_id_pc;
      end
      // =================================
      // PC update
      // =================================
      if(flush)
        PC <= next_pc_flush;
      else if (!pc_stall)
        PC <= PC + 7'b1;
      else
        PC <= PC; //stall, keep the same PC
    end//end not RST
  end//end sequential cpu logic
endmodule
