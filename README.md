# MIPS Pipelined Processor with UVM-structure Verification Testbench

Developed a 5-stage pipelined MIPS processor and built a structured verification environment in SystemVerilog inspired by UVM methodology, self-studied through Siemens onine UVM training resources.
The project focuses on validating pipeline correctness, hazard handling, and control flow through a modular testbench architecture including stimulus generation, monitoring, scoreboard checking, and functional coverage.
(An extension of the UT Austin ECE 460M Digital Systems Lab 7 ISA)

---

## Background

Lab 7 originally asked for a multi-cycle MIPS processor targeting the Basys3 FPGA board, supporting a subset of MIPS instructions plus ARM-like extensions (ADD8, RBIT, REV, SADD, SSUB). After completing the lab, I extended the project in two directions: converting the multi-cycle datapath into a 5-stage pipeline, and building a structured verification environment modeled after UVM principles.

---

## Architecture

### Multi-Cycle Baseline (`MIPS_pipeline.sv`)

The original processor uses a 5-state FSM (Fetch → Decode → Execute → Memory → Writeback) where each instruction takes 3–5 clock cycles. A slow clock divider (~3 Hz) makes instruction execution visible on the board LEDs.

### Pipelined Core (`MIPS_pipeline.sv`)

```
Cycle:   1    2    3    4    5    6    7
I1:     IF   ID   EX  MEM   WB
I2:          IF   ID   EX  MEM   WB
I3:               IF   ID   EX  MEM   WB
```

Key design decisions:

**Structural hazard** — Resolved by splitting the original unified memory into separate IMEM (combinational read) and DMEM (synchronous read/write), allowing IF and MEM to operate simultaneously.

**Data hazard (RAW)** — Handled with full forwarding:
- EX/MEM → EX: result of the immediately preceding instruction forwarded directly to ALU input
- MEM/WB → EX: result two instructions back forwarded, with priority given to the closer source
- Load-use stall: when a `LW` is immediately followed by an instruction reading the loaded register, the pipeline stalls one cycle (PC and IF/ID frozen, NOP bubble inserted into ID/EX)
- `ex_mem_rt_data` also carries the forwarded value so that `SW` writes correct data even when its source register was just written

**HALT** — PC freezes when HALT is asserted; instructions already in the pipeline drain normally.

### ISA

| Type | Instructions |
|------|-------------|
| Arithmetic | ADD, SUB, ADDI, SLT |
| Logical | AND, OR, XOR, ANDI, ORI |
| Shift | SLL, SRL |
| Memory | LW, SW |
| Immediate | LUI |
| Branch | BEQ, BNE |
| Jump | J, JAL, JR |
| ARM-like extensions | ADD8, RBIT, REV, SADD, SSUB |

---

## Verification Environment (`MIPS_Testbench.sv`)

The testbench is structured around UVM concepts adapted to plain SystemVerilog, based on concepts from Siemens EDA's UVM training (in progress).

```
┌─────────────────────────────────────────┐
│            Testbench Top                │
│                                         │
│  load_prog()   ←  Sequence              │
│  drv_*         ←  Driver               │
│  mon_*         ←  Monitor              │
│  mb_write()    ←  TLM FIFO (mailbox)   │
│  sb_check()    ←  Scoreboard           │
│  ref_alu()     ←  Golden Model         │
│  cov_*         ←  Functional Coverage  │
└─────────────────────────────────────────┘
               ↕ hierarchical reference
         MIPS_pipeline (DUT)
```

### Key components

**`load_prog()`** writes instructions directly into `DUT.IMEM` at simulation time, then triggers a reset. This makes each test case self-contained without needing separate `.hex` files, and makes instruction timing fully deterministic.

**`drv_drain()`** waits for PC to advance past the last valid instruction, then waits an additional 5 cycles for the pipeline to flush completely before reading results.

**`ref_alu()`** is a software golden model that computes expected values for all ALU operations including the extended instructions. Expected values are never hardcoded in the test cases — they always go through the reference model.

**Mailbox** (`mbox[]` register array with write/read pointers) passes transactions from monitor to scoreboard, decoupling observation from checking in the same way `uvm_tlm_fifo` does.

**Coverage** is sampled both manually (instruction type hits) and automatically via an `always @(posedge CLK)` block that watches `fwd_a_ex`, `fwd_b_ex`, `fwd_a_mem`, `fwd_b_mem`, `stall`, and `flush`.

### Test plan

| Phase | Tests |
|-------|-------|
| 1 | Reset: PC=0 after RST |
| 2 | Basic instructions: ADD, SUB, ADDI, AND, OR, XOR, SLL, SRL, LUI, SLT |
| 3 | Memory: SW followed by LW, load-use stall verification |
| 4 | Control flow: BEQ (taken), BNE (taken), J, JAL + JR |
| 5 | Hazards: EX→EX forwarding, MEM→EX forwarding, load-use stall |
| 6 | Extended instructions: ADD8, RBIT, REV, SADD (normal + saturate), SSUB (saturate) |
| 7 | HALT: PC freezes, releases correctly |

### Sample output

```
===== PHASE 5: Hazard Tests =====

  [TEST] EX→EX Forwarding: addi $3,7 then add $2=$3
  [MONITOR] 應看到 fwdA=10（EX forwarding）:
  cyc=89  PC=01 | IF=00000000 ID=20030007 | fwdA=10 fwdB=00 stall=0 flush=0 | EX_rd=03 EX_res=00000007
  [PASS] FORWARDING   got=0x00000007

  [TEST] Load-Use Stall: lw $3 then immediately use $3
  [MONITOR] 應看到 stall=1：
  cyc=102 PC=01 | IF=00000000 ID=8c030000 | fwdA=00 fwdB=00 stall=1 flush=0 | EX_rd=03 EX_res=00000000
  [PASS] LOAD-USE     got=0x12345678

============================================================
  SCOREBOARD REPORT
  Total : 28  |  PASS : 28  |  FAIL : 0
  *** ALL TESTS PASSED ***

============================================================
  COVERAGE REPORT
  Instruction coverage : 22/22 (100.0%)
  Hazard coverage:
    EX/MEM→EX forwarding : HIT
    MEM/WB→EX forwarding : HIT
    Load-use stall       : HIT
    Branch/Jump flush    : HIT
    HALT                 : HIT
    RESET                : HIT
============================================================
```

---

## Files

```
MIPS_pipeline.sv       — 5-stage pipelined processor
MIPS_b.sv              — multi-cycle baseline (lab original)
MIPS_Testbench.sv      — DV-style verification environment
regfile.v              — register file (shared)
seven_seg.v            — 7-segment display driver (FPGA)
top_partA.v            — FPGA top for Part A (LED rotation)
top_partB.v            — FPGA top for Part B (switch + 7-seg)
lab7_constraints.xdc   — Basys3 pin constraints
program_partA.hex      — LED rotation machine code
program_partB.hex      — Part B test program machine code
```

---

## Tools

- **Simulation**: ModelSim (run.do stript including `vlog -sv`, `vsim -voptargs=+acc`)

---

## Status

- [x] Multi-cycle MIPS (ECE460M lab7)
- [x] 5-stage pipeline core
- [x] Forwarding (EX/MEM→EX, MEM/WB→EX)
- [x] Load-use hazard stall
- [x] DV testbench with scoreboard and coverage
- [ ] Complete Siemens UVM training and port testbench to true UVM

---

## Notes

The UVM-style structure is based on basic concepts from Siemens EDA's online training, not a complete UVM implementation. Key limitations compared to real UVM: no class-based objects (uses reg arrays as mailbox), no virtual interfaces, no factory/config_db, and no parallel threads (monitor and driver run sequentially in the same initial block). The goal was to apply the conceptual layering — sequence/driver/monitor/scoreboard/coverage — while staying within synthesizable SystemVerilog constraints for simulation.
