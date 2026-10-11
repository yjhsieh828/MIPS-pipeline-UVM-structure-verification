# MIPS Pipelined Processor with Branch Prediction, Return Address Stack, and UVM-structure Verification Testbench

Developed a 5-stage pipelined MIPS processor and built a structured verification environment in SystemVerilog inspired by UVM methodology, self-studied through Siemens onine UVM training resources.
The project focuses on validating pipeline correctness, hazard handling, and control flow through a modular testbench architecture including stimulus generation, monitoring, scoreboard checking, and functional coverage.
An updated version further implemented a 64-entry branch target buffer with two-bit branch direction predictor, and a 8-entry return address stack. In this version, CPI speed up 2.6x with total area rose 1.56x.
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

**Branch and return prediction**
- ***BTB/BHT:*** `TABLE_SIZE=6` provides 64 tagged entries and two-bit saturating direction counters. ID lookup uses the instruction's saved `PC+1`. Counters initialize to strongly not taken (`00`).
- ***Conditional branches:*** a predicted taken branch redirects PC in ID and discards the sequential fetch. EX resolves the outcome, updates the predictor, and flushes younger instructions on a direction mismatch.
- ***RAS:*** `RAS_SIZE=3` allocates eight return-address slots. JAL pushes a return address in EX; a nonempty RAS predicts `JR $31` in ID. EX compares the prediction with the forwarded register target and redirects on a mismatch or an unpredicted JR. The current pointer and push guard permit seven outstanding pushes.
- ***J/JAL:*** direct jumps redirect in EX and flush younger instructions. JAL writes its saved `PC+1` to `$31` through the pipeline.
- ***Reset:*** RST clears pipeline/register state and the RAS. BTB/BHT state initializes at simulation startup and is not cleared by RST, so predictor history can persist between test phases.

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
| 8 | Recursive Fibonacci: final `$2 = fib(7) = 13` |

### Sample output

```
# ===== PHASE 7: HALT =====
#   [MON] reg2_out=0x00000001  cyc=551  PC=1
#   before HALT: PC=0  $2=0x00000001
#   freeze: PC=2
#   [PASS] HALT 10 cycles PC frozen
#   [PASS] HALT        got=0x00000001
#   release HALT: PC=2
# 
# ============================================================
#   SCOREBOARD REPORT
#   Total : 29  |  PASS : 29  |  FAIL : 0
#   *** ALL TESTS PASSED ***
# ============================================================
# 
# ============================================================
#   COVERAGE REPORT
#   Instruction coverage : 26/26 (100.0%)
#     ADD        :  HIT
#     ADDI       :  HIT
#     SUB        :  HIT
#     AND        :  HIT
#     OR         :  HIT
#     XOR        :  HIT
#     SLL        :  HIT
#     SRL        :  HIT
#     LUI        :  HIT
#     LW         :  HIT
#     SW         :  HIT
#     BEQ        :  HIT
#     BNE        :  HIT
#     J          :  HIT
#     JAL        :  HIT
#     JR         :  HIT
#     SLT        :  HIT
#     RBIT       :  HIT
#     REV        :  HIT
#     ADD8       :  HIT
#     SADD       :  HIT
#     SSUB       :  HIT
#     RESET      :  HIT
#     HALT       :  HIT
#     FORWARDING :  HIT
#     LOAD-USE   :  HIT
#   Hazard coverage:
#     EX/MEM->EX forwarding :  HIT
#     MEM/WB->EX forwarding :  HIT
#     Load-use stall       :  HIT
#     Branch/Jump flush    :  HIT
#     HALT                 :  HIT
#     RESET                :  HIT
#   Branch/Jump statistics at FIBONACCI:
#     Total branches (taken/all): 20 / 41
#     Total BTB predict accuracy: 48.8%
#     Total jump calls          : 41
# ============================================================
# [SUMMARY] total_fail=0
```

### Recursive Fibonacci workload

The program passes `n=7` in `$4`, initializes software stack pointer `$29` to word index 90, and calls `fib` at instruction index 4. Each non-base invocation allocates a *three-word frame* containing its return address, saved argument, and `fib(n-1)` result. The base case returns `n` for `n<2`; other invocations add `fib(n-1)` and `fib(n-2)`.

The workload exercises nested JAL/JR control flow, stack loads/stores, forwarding, load-use stalls, conditional branch prediction, and return prediction. Completion is detected when a JR in EX targets the main return index 3, followed by five wait cycles and HALT. The scoreboard checks the final result rather than every executed instruction.

The supplied simulation excerpt reports:
```text
[PASS] FIBONACCI   got=0x0000000d
[STATS] fib(7): branches=41 taken=20 (48.8%) calls(JAL)=41
---

## Files

```
MIPS_pipeline.sv       — 5-stage pipelined processor
MIPS_Testbench.sv      — DV-style verification environment

MIPS_b.sv              — multi-cycle baseline (lab original)
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

- **Simulation**: ModelSim (run.do script including `vlog -sv`, `vsim -voptargs=+acc`)
- **Synthesis** : Genus (syn.tcl for pre-APR area and power extimation)

---

## Status

- [x] Multi-cycle MIPS (ECE460M lab7)
- [x] 5-stage pipeline core
- [x] Forwarding (EX/MEM→EX, MEM/WB→EX)
- [x] Load-use hazard stall
- [x] BTB/BHT branch prediction and RAS return prediction
- [x] DV testbench with scoreboard and coverage
- [x] Recursive Fibonacci workload; supplied excerpt shows `fib(7)=13`
- [ ] Complete Siemens UVM training and port testbench to true UVM

---

## Notes

The UVM-style structure is based on basic concepts from Siemens online training, served as a learning project implemented in plain SystemVerilog. The testbench adopts key UVM concepts such as modular components, stimulus generation, monitoring, and checking. Key limitations compared to real UVM: no class-based objects (uses reg arrays as mailbox), no virtual interfaces, no factory/config_db, and no parallel threads (monitor and driver run sequentially in the same initial block). The goal was to apply the conceptual layering — sequence/driver/monitor/scoreboard/coverage — while staying within synthesizable SystemVerilog constraints for simulation.
