#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "tube_decode.h"
#include "em_65816.h"

// ====================================================================
// Type Defs
// ====================================================================

typedef enum {
   IMP,
   IMPA,
   BRA,
   IMM,
   ZP,
   ZPX,
   ZPY,
   INDX,
   INDY,
   IND,
   ABS,
   ABSX,
   ABSY,
   IND16,
   IND1X,
   ZPR,
   SR,
   ISY,
   IDL,
   IDLY,
   ABL,
   ALX,
   IAL,
   BRL,
   BM
} AddrMode ;

typedef enum {
   READOP,
   WRITEOP,
   RMWOP,
   TSBTRBOP,
   BRANCHOP,
   OTHER
} OpType;

typedef struct {
   int len;
   const char *fmt;
} AddrModeType;

typedef uint32_t operand_t;

typedef int ea_t;

typedef struct {
   const char *mnemonic;
   int undocumented;
   AddrMode mode;
   int cycles;
   OpType optype;
   int (*emulate)(operand_t, ea_t);
   int len;
   int m_extra;
   int x_extra;
   const char *fmt;
} InstrType;


// ====================================================================
// Static variables
// ====================================================================

#define OFFSET_B    2
#define OFFSET_A    4
#define OFFSET_X    9
#define OFFSET_Y   16
#define OFFSET_SH  24
#define OFFSET_SL  26
#define OFFSET_N   31
#define OFFSET_V   35
#define OFFSET_MS  39
#define OFFSET_XS  43
#define OFFSET_D   47
#define OFFSET_I   51
#define OFFSET_Z   55
#define OFFSET_C   59
#define OFFSET_E   63
#define OFFSET_PB  68
#define OFFSET_DB  74
#define OFFSET_DP  80
#define OFFSET_END 84

static const char default_state[] = "A=???? X=???? Y=???? SP=???? N=? V=? M=? X=? D=? I=? Z=? C=? E=? PB=?? DB=?? DP=????";

static int c02;
static int bbctube;

static InstrType *instr_table;

AddrModeType addr_mode_table[] = {
   {1,    "%1$s"},                     // IMP
   {1,    "%1$s A"},                   // IMPA
   {2,    "%1$s %2$s"},                // BRA
   {2,    "%1$s #%2$02X"},             // IMM
   {2,    "%1$s %2$02X"},              // ZP
   {2,    "%1$s %2$02X,X"},            // ZPX
   {2,    "%1$s %2$02X,Y"},            // ZPY
   {2,    "%1$s (%2$02X,X)"},          // INDX
   {2,    "%1$s (%2$02X),Y"},          // INDY
   {2,    "%1$s (%2$02X)"},            // IND
   {3,    "%1$s %3$02X%2$02X"},        // ABS
   {3,    "%1$s %3$02X%2$02X,X"},      // ABSX
   {3,    "%1$s %3$02X%2$02X,Y"},      // ABSY
   {3,    "%1$s (%3$02X%2$02X)"},      // IND1
   {3,    "%1$s (%3$02X%2$02X,X)"},    // IND1X
   {3,    "%1$s %2$02X,%3$s"},         // ZPR
   {2,    "%1$s %2$02X,S"},            // SR
   {2,    "%1$s (%2$02X,S),Y"},        // ISY
   {2,    "%1$s [%2$02X]"},            // IDL
   {2,    "%1$s [%2$02X],Y"},          // IDLY
   {4,    "%1$s %4$02X%3$02X%2$02X"},  // ABL
   {4,    "%1$s %4$02X%3$02X%2$02X,X"},// ABLX
   {3,    "%1$s [%3$02X%2$02X]"},      // IAL
   {3,    "%1$s %2$s"},                // BRL
   {3,    "%1$s %2$02X,%3$02X"}        // BM
};

static const char *fmt_imm16 = "%1$s #%3$02X%2$02X";

// 6502 registers: -1 means unknown
static int A = -1;
static int X = -1;
static int Y = -1;

static int SH = -1;
static int SL = -1;

static int PC = -1;

// 65C816 additional registers: -1 means unknown
static int B  = -1; // Accumulator bits 15..8
static int DP = -1; // 16-bit Direct Page Register
static int DB = -1; // 8-bit Data Bank Register
static int PB = -1; // 8-bit Program Bank Register

// 6502 flags: -1 means unknown
static int N = -1;
static int V = -1;
static int D = -1;
static int I = -1;
static int Z = -1;
static int C = -1;

// 65C816 additional flags: -1 means unknown
static int MS = -1; // Accumulator and Memeory Size Flag
static int XS = -1; // Index Register Size Flag
static int E =  -1; // Emulation Mode Flag, updated by XCE

// indicate state prediction failed
static int failflag = 0;

// 16MB Main Memory
static int memory[0x1000000];

static char *x1_ops[] = {
   "CPX",
   "CPY",
   "DEX",
   "DEY",
   "INX",
   "INY",
   "LDX",
   "LDY",
   "PHX",
   "PHY",
   "PLX",
   "PLY",
   "STX",
   "STY",
   NULL
};

static char *m1_ops[] = {
   "ADC",
   "AND",
   "BIT",
   "CMP",
   "EOR",
   "LDA",
   "ORA",
   "PHA",
   "PLA",
   "SBC",
   "STA",
   "STZ",
   NULL
};

static char *m2_ops[] = {
   "ASL",
   "DEC",
   "INC",
   "LSR",
   "ROL",
   "ROR",
   "TSB",
   "TRB",
   NULL
};

// ====================================================================
// Forward declarations
// ====================================================================

static InstrType instr_table_65c816[];

static void emulation_mode_on();
static void emulation_mode_off();
static int op_STA(operand_t operand, ea_t ea);
static int op_STX(operand_t operand, ea_t ea);
static int op_STY(operand_t operand, ea_t ea);

// ====================================================================
// Helper Methods
// ====================================================================

static void memory_read(int data, int ea) {
   if (ea >= 0 && ea < 0x8000) {
      if (memory[ea] >=0 && memory[ea] != data) {
         printf("memory modelling failed at %04x: expected %02x, actual %02x\n", ea, memory[ea], data);
         failflag |= 1;
      }
      memory[ea] = data;
   }
   if (bbctube && ea >= 0xfee0 && ea <= 0xfee7) {
      tube_read(ea & 7, data);
   }
}

static void memory_write(int data, int ea) {
   if (ea >= 0) {
      // Data can be negarive, which means the memory becomes undefined again
      //printf("memory write: %06x = %02x\n", ea, data);
      memory[ea] = data;
   }
   if (bbctube && ea >= 0xfee0 && ea <= 0xfee7) {
      tube_write(ea & 7, data);
   }
}

static int compare_FLAGS(int operand) {
   if (N >= 0) {
      if (N != ((operand >> 7) & 1)) {
         return 1;
      }
   }
   if (V >= 0) {
      if (V != ((operand >> 6) & 1)) {
         return 1;
      }
   }
   if (E == 0 && MS >= 0) {
      if (MS != ((operand >> 5) & 1)) {
         return 1;
      }
   }
   if (E == 0 && XS >= 0) {
      if (XS != ((operand >> 4) & 1)) {
         return 1;
      }
   }
   if (D >= 0) {
      if (D != ((operand >> 3) & 1)) {
         return 1;
      }
   }
   if (I >= 0) {
      if (I != ((operand >> 2) & 1)) {
         return 1;
      }
   }
   if (Z >= 0) {
      if (Z != ((operand >> 1) & 1)) {
         return 1;
      }
   }
   if (C >= 0) {
      if (C != ((operand >> 0) & 1)) {
         return 1;
      }
   }
   return 0;
}

static void check_FLAGS(int operand) {
   failflag |= compare_FLAGS(operand);
}

static void set_FLAGS(int operand) {
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   if (E == 0) {
      MS = (operand >> 5) & 1;
      XS = (operand >> 4) & 1;
   } else {
      MS = 1;
      XS = 1;
   }
   D = (operand >> 3) & 1;
   I = (operand >> 2) & 1;
   Z = (operand >> 1) & 1;
   C = (operand >> 0) & 1;
}

static void set_NZ_unknown() {
   N = -1;
   Z = -1;
}

static void set_NZC_unknown() {
   N = -1;
   Z = -1;
   C = -1;
}

static void set_NVZC_unknown() {
   N = -1;
   V = -1;
   Z = -1;
   C = -1;
}

static void set_NZ8(int value) {
   N = (value >> 7) & 1;
   Z = (value & 0xff) == 0;
}

static void set_NZ16(int value) {
   N = (value >> 15) & 1;
   Z = (value & 0xffff) == 0;
}

static void set_NZ_unknown_width(int value) {
   // Don't know which bit is the sign bit
   int s15 = (value >> 15) & 1;
   int s7 = (value >> 7) & 1;
   if (s7 == s15) {
      // both choices of sign bit are the same
      N = s7;
   } else {
      // possible sign bits differ, so N must become undefined
      N = -1;
   }
   // Don't know how many bits to check for any ones
   if ((value & 0xff00) == 0) {
      // no high bits set, so base Z on the low bits
      Z = (value & 0xff) == 0;
   } else {
      // some high bits set, so Z must become undefined
      Z = -1;
   }
}

static void set_NZ_XS(int value) {
   if (XS < 0) {
      set_NZ_unknown_width(value);
   } else if (XS == 0) {
      set_NZ16(value);
   } else {
      set_NZ8(value);
   }
}

static void set_NZ_MS(int value) {
   if (MS < 0) {
      set_NZ_unknown_width(value);
   } else if (MS == 0) {
      set_NZ16(value);
   } else {
      set_NZ8(value);
   }
}

static void set_NZ_AB(int A, int B) {
   if (MS > 0) {
      // 8-bit
      if (A >= 0) {
         set_NZ8(A);
      } else {
         set_NZ_unknown();
      }
   } else if (MS == 0) {
      // 16-bit
      if (A >= 0 && B >= 0) {
         set_NZ16((B << 8) + A);
      } else {
         // TODO: the behaviour when A is known and B is unknown could be improved
         set_NZ_unknown();
      }
   } else {
      // width unknown
      if (A >= 0 && B >= 0) {
         set_NZ_unknown_width((B << 8) + A);
      } else {
         set_NZ_unknown();
      }
   }
}

static void pop8(int value) {
   // Increment the low byte of SP
   if (SL >= 0) {
      SL = (SL + 1) & 0xFF;
   }
   // Increment the high byte of SP, in certain cases
   if (E == 1) {
      // In emulation mode, force SH to 1
      SH = 1;
   } else if (E == 0) {
      // In native mode, increment SH if SL has wrapped to 0
      if (SH >= 0) {
         if (SL < 0) {
            SH = -1;
         } else if (SL == 0) {
            SH = (SH + 1) & 0xFF;
         }
      }
   } else {
      SH = -1;
   }
   // Handle the memory access
   if (SL >= 0 && SH >= 0) {
      memory_read(value & 0xff, (SH << 8) + SL);
   }
}

static void push8(int value) {
   // Handle the memory access
   if (SL >= 0 && SH >= 0) {
      memory_write(value & 0xff, (SH << 8) + SL);
   }
   // Decrement the low byte of SP
   if (SL >= 0) {
      SL = (SL - 1) & 0xFF;
   }
   // Decrement the high byte of SP, in certain cases
   if (E == 1) {
      // In emulation mode, force SH to 1
      SH = 1;
   } else if (E == 0) {
      // In native mode, increment SH if SL has wrapped to 0
      if (SH >= 0) {
         if (SL < 0) {
            SH = -1;
         } else if (SL == 0xFF) {
            SH = (SH - 1) & 0xFF;
         }
      }
   } else {
      SH = -1;
   }
}

static void pop16(int value) {
   pop8(value);
   pop8(value >> 8);
}

static void push16(int value) {
   push8(value >> 8);
   push8(value);
}

static void popXS(int value) {
   if (XS < 0) {
      SL = -1;
      SH = -1;
   } else if (XS == 0) {
      pop16(value);
   } else {
      pop8(value);
   }
}

static void popMS(int value) {
   if (MS < 0) {
      SL = -1;
      SH = -1;
   } else if (MS == 0) {
      pop16(value);
   } else {
      pop8(value);
   }
}

static void pushXS(int value) {
   if (XS < 0) {
      SL = -1;
      SH = -1;
   } else if (XS == 0) {
      push16(value);
   } else {
      push8(value);
   }
}

static void pushMS(int value) {
   if (MS < 0) {
      SL = -1;
      SH = -1;
   } else if (MS == 0) {
      push16(value);
   } else {
      push8(value);
   }
}

static void interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction, int pc_offset) {
   int i;
   int pb;
   if (num_cycles == 7) {
      // We must be in emulation mode
      emulation_mode_on();
      i = 2;
      pb = PB;
   } else {
      // We must be in native mode
      pb = sample_q[2].data;
      emulation_mode_off();
      i = 3;
      pb = sample_q[2].data;
   }
   // Parse the bus cycles
   // E=0 <opcode> <op1> <write pbr> <write pch> <write pcl> <write p> <read rst> <read rsth>
   // E=1 <opcode> <op1>             <write pch> <write pcl> <write p> <read rst> <read rsth>
   int pc     = (sample_q[i].data << 8) + sample_q[i + 1].data;
   int flags  = sample_q[i + 2].data;
   int vector = (sample_q[i + 4].data << 8) + sample_q[i + 3].data;
   // Update the address of the interruted instruction
   if (pb >= 0) {
      instruction->pb = pb;
   }
   instruction->pc = (pc - pc_offset) & 0xffff;
   // Stack the PB/PC/FLags (for memory modelling)
   if (E == 0) {
      push8(pb);
   }
   push16(pc);
   push8(flags);
   // Validate the flags
   check_FLAGS(flags);
   // And make them consistent
   set_FLAGS(flags);
   // Setup expected state for the ISR
   I = 1;
   D = 0;
   PB = 0x00;
   PC = vector;
}

static int count_cycles_without_sync(sample_t *sample_q, int intr_seen) {

   printf("VPA/VDA must be connected in 65816 mode\n");
   exit(1);

   return 0;
}

static int count_cycles_with_sync(sample_t *sample_q) {
   if (sample_q[0].type == OPCODE) {
      for (int i = 1; i < DEPTH; i++) {
         if (sample_q[i].type == LAST) {
            return 0;
         }
         if (sample_q[i].type == OPCODE) {
            return i;
         }
      }
   }
   return 1;
}

// A set of actions to take if emulation mode enabled
static void emulation_mode_on() {
   if (E == 0) {
      failflag = 1;
   }
   MS = 1;
   XS = 1;
   if (X >= 0) {
      X &= 0x00ff;
   }
   if (Y >= 0) {
      Y &= 0x00ff;
   }
   SH = 0x01;
   E = 1;
}

// A set of actions to take if emulation mode enabled
static void emulation_mode_off() {
   if (E == 1) {
      failflag = 1;
   }
   E = 0;
}

static void check_and_set_ms(int val) {
   if (MS >= 0 &&  MS != val) {
      failflag = 1;
   }
   MS = val;
   // Evidence of MS = 0 implies E = 0
   if (MS == 0) {
      emulation_mode_off();
   }
}

static void check_and_set_xs(int val) {
   if (XS >= 0 &&  XS != val) {
      failflag = 1;
   }
   XS = val;
   // Evidence of XS = 0 implies E = 0
   if (XS == 0) {
      emulation_mode_off();
   }
}

// ====================================================================
// Public Methods
// ====================================================================

static void em_65816_init(cpu_t cpu_type, int e_flag, int sp_reg, int decode_bbctube) {
   switch (cpu_type) {
   case CPU_65C816:
      c02 = 1;
      instr_table = instr_table_65c816;
      break;
   default:
      printf("em_65816_init called with unsupported cpu_type (%d)\n", cpu_type);
      exit(1);
   }
   bbctube = decode_bbctube;
   if (e_flag >= 0) {
      E  = e_flag & 1;
      if (E) {
         emulation_mode_on();
      } else {
         emulation_mode_off();
      }
   }
   if (sp_reg >= 0) {
      SL = sp_reg & 0xff;
      SH = (sp_reg >> 8) & 0xff;
   }
   InstrType *instr = instr_table;
   for (int i = 0; i < 256; i++) {
      // Compute the extra cycles for the 816 when M=0 and/or X=0
      instr->m_extra = 0;
      instr->x_extra = 0;
      if (instr->mode != IMPA) {
         // add 1 cycle if m=0: ADC, AND, BIT, CMP, EOR, LDA, ORA, PHA, PLA, SBC, STA, STZ
         for (int j = 0; m1_ops[j]; j++) {
            if (!strcmp(instr->mnemonic, m1_ops[j])) {
               instr->m_extra++;
               break;
            }
         }
         // add 2 cycles if m=0 (NOT the implied ones): ASL, DEC, INC, LSR, ROL, ROR, TRB, TSB
         for (int j = 0; m2_ops[j]; j++) {
            if (!strcmp(instr->mnemonic, m2_ops[j])) {
               instr->m_extra += 2;
               break;
            }
         }
         // add 1 cycle if x=0: CPX, CPY, LDX, LDY, STX, STY, PLX, PLY, PHX, PHY
         for (int j = 0; x1_ops[j]; j++) {
            if (!strcmp(instr->mnemonic, x1_ops[j])) {
               instr->x_extra++;
               break;
            }
         }
      }
      // Copy the length and format from the address mode, for efficiency
      instr->len = addr_mode_table[instr->mode].len;
      instr->fmt = addr_mode_table[instr->mode].fmt;
      //printf("%02x %d %d %d\n", i, instr->m_extra, instr->x_extra, instr->len);
      instr++;
   }
   for (int i = 0; i < sizeof(memory) / sizeof(int); i++) {
      memory[i] = -1;
   }
}

static int em_65816_match_interrupt(sample_t *sample_q, int num_samples) {
   // Check we have enough valid samples
   if (num_samples < 7) {
      return 0;
   }
   // In emulation mode an interupt will write PCH, PCL, PSW in bus cycles 2,3,4
   // In native mode an interupt will write PBR, PCH, PCL, PSW in bus cycles 2,3,4,5
   //
   // TODO: the heuristic only works in emulation mode
   if (sample_q[0].rnw >= 0) {
      // If we have the RNW pin connected, then just look for these three writes in succession
      // Currently can't detect a BRK being interrupted
      if (sample_q[0].data == 0x00) {
         return 0;
      }
      if (sample_q[2].rnw == 0 && sample_q[3].rnw == 0 && sample_q[4].rnw == 0) {
         return 1;
      }
   } else {
      // If not, then we use a heuristic, based on what we expect to see on the data
      // bus in cycles 2, 3 and 4, i.e. PCH, PCL, PSW
      if (sample_q[2].data == ((PC >> 8) & 0xff) && sample_q[3].data == (PC & 0xff)) {
         // Now test unused flag is 1, B is 0
         if ((sample_q[4].data & 0x30) == 0x20) {
            // Finally test all other known flags match
            if (!compare_FLAGS(sample_q[4].data)) {
               // Matched PSW = NV-BDIZC
               return 1;
            }
         }
      }
   }
   return 0;
}

static int em_65816_count_cycles(sample_t *sample_q, int intr_seen) {
   if (sample_q[0].type == UNKNOWN) {
      return count_cycles_without_sync(sample_q, intr_seen);
   } else {
      return count_cycles_with_sync(sample_q);
   }
}

static void em_65816_reset(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   instruction->pc = -1;
   A = -1;
   X = -1;
   Y = -1;
   SH = -1;
   SL = -1;
   N = -1;
   V = -1;
   D = -1;
   Z = -1;
   C = -1;
   I = 1;
   D = 0;
   // Extra 816 regs
   B = -1;
   DP = 0;
   PB = 0;
   // Extra 816 flags
   E = 1;
   emulation_mode_on();
   // Program Counter
   PC = (sample_q[num_cycles - 1].data << 8) + sample_q[num_cycles - 2].data;
}

static void em_65816_interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   interrupt(sample_q, num_cycles, instruction, 0);
}

static void em_65816_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction) {

   // Unpack the instruction bytes
   int opcode = sample_q[0].data;

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   int opcount = 0;
   // Immediate operands can be 16-bit
   if (instr->mode == IMM) {
      if (instr->m_extra) {
         if (num_cycles == 3) {
            opcount = 1;
            check_and_set_ms(0); // infer 16-bit mode for A/M
         } else {
            check_and_set_ms(1); // infer 8-bit mode for A/M
         }
      }
      if (instr->x_extra) {
         if (num_cycles == 3) {
            opcount = 1;
            check_and_set_xs(0); // infer 16-bit mode for X/Y
         } else {
            check_and_set_xs(1); // infer 8-bit mode for X/Y
         }
      }
   }
   opcount += instr->len - 1;

   int op1 = (opcount < 1) ? 0 : sample_q[1].data;

   int op2 = (opcount < 2) ? 0 : sample_q[2].data;

   int op3 = (opcount < 3) ? 0 : sample_q[(opcode == 0x22) ? 5 : 3].data;

   // Save the instruction state
   instruction->opcode  = opcode;
   instruction->op1     = op1;
   instruction->op2     = op2;
   instruction->op3     = op3;
   instruction->opcount = opcount;

   // Fill in the current PB/PC value
   if (opcode == 0x00 || opcode == 0x02) {
      // BRK or COP - handle in the same way as an interrupt
      // Now just pass BRK onto the interrupt handler
      interrupt(sample_q, num_cycles, instruction, 2);
      // And we are done
      return;
   } else if (opcode == 0x20) {
      // JSR: <opcode> <op1> <op2> <read dummy> <write pch> <write pcl>
      instruction->pc = (((sample_q[4].data << 8) + sample_q[5].data) - 2) & 0xffff;
      instruction->pb = PB;
   } else if (opcode == 0x22) {
      // JSL: <opcode> <op1> <op2> <write pbr> <read dummy> <op3> <write pch> <write pcl>
      instruction->pc = (((sample_q[6].data << 8) + sample_q[7].data) - 3) & 0xffff;
      instruction->pb = sample_q[3].data;
   } else {
      instruction->pc = PC;
      instruction->pb = PB;
   }

   uint32_t operand;
   if (instr->optype == RMWOP) {
      // e.g. <opcode> <op1> <op2> <read old> <dummy> <write new>
      // 2/12/2020: on Beeb816 the <read old> cycle seems hard to sample
      // reliably with the FX2, so lets use the <dummy> instead.
      // E=0 - Dummy is a read of the same data (albeit with VPA/VDA=00)
      // E=1 - Dummy is a write of the same data
      // TODO: handle 16-bits
      if (E == 1) {
         operand = sample_q[num_cycles - 2].data;
      } else {
         operand = sample_q[num_cycles - 3].data;
      }
   } else if (instr->optype == BRANCHOP) {
      // the operand is true if branch taken
      operand = (num_cycles != 2);
   } else if (opcode == 0x20) {
      // JSR: the operand is the data pushed to the stack (PCH, PCL)
      // <opcode> <op1> <op2> <read dummy> <write pch> <write pcl>
      operand = (sample_q[4].data << 8) + sample_q[5].data;
   } else if (opcode == 0x22) {
      // JSL: the operand is the data pushed to the stack (PCB, PCH, PCL)
      // <opcode> <op1> <op2> <write pbr> <read dummy> <op3> <write pch> <write pcl>
      operand = (sample_q[3].data << 16) + (sample_q[6].data << 8) + sample_q[7].data;
   } else if (opcode == 0x40) {
      // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
      // E=0: <opcode> <op1> <read dummy> <read p> <read pcl> <read pch> <read pbr>
      // E=1: <opcode> <op1> <read dummy> <read p> <read pcl> <read pch>
      operand = (sample_q[5].data << 16) +  (sample_q[4].data << 8) + sample_q[3].data;
      if (num_cycles == 6) {
         emulation_mode_on();
      } else {
         emulation_mode_off();
         operand |= (sample_q[6].data << 24);
      }
   } else if (opcode == 0x60) {
      // RTS: the operand is the data pulled from the stack (PCL, PCH)
      // <opcode> <op1> <read dummy> <read pcl> <read pch> <read dummy>
      operand = (sample_q[4].data << 8) + sample_q[3].data;
   } else if (opcode == 0x6B) {
      // RTL: the operand is the data pulled from the stack (PCL, PCH, PBR)
      // <opcode> <op1> <read dummy> <read pcl> <read pch> <read pbr>
      operand = (sample_q[5].data << 16) + (sample_q[4].data << 8) + sample_q[3].data;
   } else if (instr->mode == IMM) {
      // Immediate addressing mode: the operand is the 2nd byte of the instruction
      operand = (op2 << 8) + op1;
   } else if (instr->optype == TSBTRBOP) {
      // For TSB/TRB, <opcode> <op1> <read> <dummy> <write> the operand is the <read>
      // TODO: handle 16-bits
      operand = sample_q[num_cycles - 3].data;
   } else {
      // default to using the last bus cycle(s) as the operand
      // special case PHD (0B) / PLD (2B) as these are always 16-bit
      if ((instr->m_extra && (MS == 0)) || (instr->x_extra && (XS == 0)) || opcode == 0x0B || opcode == 0x2B)  {
         // 16-bit operation
         if (opcode == 0x48 || opcode == 0x5A || opcode == 0xDA || opcode == 0x0B) {
            // PHA/PHX/PHY/PHD push high byte followed by low byte
            operand = sample_q[num_cycles - 1].data + (sample_q[num_cycles - 2].data << 8);
         } else {
            // all other 16-bit ops are low byte then high byer
            operand = sample_q[num_cycles - 2].data + (sample_q[num_cycles - 1].data << 8);
         }
      } else {
         // 8-bit operation
         operand = sample_q[num_cycles - 1].data;
      }
   }

   // For instructions that read or write memory, we need to work out the effective address
   // Note: not needed for stack operations, as S is used directly
   int ea = -1;
   int index;
   switch (instr->mode) {
   case ZP:
      ea = op1;
      break;
   case ZPX:
   case ZPY:
      index = instr->mode == ZPX ? X : Y;
      if (index >= 0) {
         ea = (op1 + index) & 0xff;
      }
      break;
   case INDY:
      // <opcpde> <op1> <addrlo> <addrhi> [ <page crossing>] <<operand> [ <extra cycle in dec mode> ]
      index = Y;
      if (index >= 0) {
         ea = (sample_q[3].data << 8) + sample_q[2].data;
         ea = (ea + index) & 0xffff;
      }
      break;
   case INDX:
      // <opcpde> <op1> <dummy> <addrlo> <addrhi> <operand> [ <extra cycle in dec mode> ]
      ea = (sample_q[4].data << 8) + sample_q[3].data;
      break;
   case IND:
      // <opcpde> <op1> <addrlo> <addrhi> <operand> [ <extra cycle in dec mode> ]
      ea = (sample_q[3].data << 8) + sample_q[2].data;
      break;
   case ABS:
      ea = op2 << 8 | op1;
      break;
   case ABSX:
   case ABSY:
      index = instr->mode == ABSX ? X : Y;
      if (index >= 0) {
         ea = ((op2 << 8 | op1) + index) & 0xffff;
      }
      break;
   case BRA:
      if (PC > 0) {
         ea = (PC + ((int8_t)(op1)) + 2) & 0xffff;
      }
      break;
   case BRL:
      if (PC > 0) {
         ea = (PC + ((int16_t)((op2 << 8) + op1)) + 3) & 0xffff;
      }
      break;
   case SR:
      if (SL >= 0 && SH >= 0) {
         ea = ((SH << 8) + SL + op1) & 0xffff;
      }
      break;
   default:
      break;
   }

   if (instr->emulate) {

      // Determine memory access size
      int size = instr->x_extra ? XS : instr->x_extra ? MS : 1;

      // Model memory reads
      if (ea >= 0 && (instr->optype == READOP || instr->optype == RMWOP || instr->optype == TSBTRBOP)) {
         int oplo = operand < 0 ? -1 : (operand & 0xff);
         int ophi = operand < 0 ? -1 : ((operand >> 8) & 0xff);
         if (size == 0) {
            memory_read(oplo,  ea);
            memory_read(ophi, (ea + 1) & 0xffff);
         } else if (size > 0) {
            memory_read(oplo, ea);
         }
      }

      // Execute the instruction specific function
      int result = instr->emulate(operand, ea);

      // Model memory writes
      if (ea >= 0 && (instr->optype == WRITEOP || instr->optype == RMWOP || instr->optype == TSBTRBOP)) {
         // STA STX STY STZ
         // INC DEX ASL LSR ROL ROR
         // TSB TRB
         // (if A is unknown, then TSB/TRB are uiknown)
         // (if C is unlnown, then ROL/ROR are unknown)
         // These cases could be eliminated if we snoopedthe result of Read-Modify-Weith
         int reslo = result < 0 ? -1 : (result & 0xff);
         int reshi = result < 0 ? -1 : ((result >> 8) & 0xff);
         if (size == 0) {
            memory_write(reslo,  ea);
            memory_write(reshi, (ea + 1) & 0xffff);
         } else if (size > 0) {
            memory_write(reslo, ea);
         }
      }
   }

   // Look for control flow changes and update the PC
   if (opcode == 0x40) {
      // E=0: <opcode> <op1> <read dummy> <read p> <read pcl> <read pch> <read pbr>
      // E=1: <opcode> <op1> <read dummy> <read p> <read pcl> <read pch>
      PC = sample_q[4].data | (sample_q[5].data << 8);
      if (E == 0) {
         PB = sample_q[6].data;
      }
   } else if (opcode == 0x6c || opcode == 0x7c) {
      // JMP (ind), JMP (ind, X)
      PC = (sample_q[num_cycles - 1].data << 8) | sample_q[num_cycles - 2].data;
   } else if (opcode == 0x20 || opcode == 0x4c) {
      // JSR abs, JMP abs
      PC = (op2 << 8) | op1;
   } else if (opcode == 0x22 || opcode == 0x5c) {
      // JSL long, JMP long
      PB = op3;
      PC = (op2 << 8) | op1;
   } else if (PC < 0) {
      // PC value is not known yet, everything below this point is relative
      PC = -1;
   } else if (opcode == 0x80 || opcode == 0x82) {
      // BRA / BRL
      PC = ea;
   } else if ((opcode & 0x1f) == 0x10 && num_cycles != 2) {
      // BXX: if taken
      PC = ea;
   } else {
      // Otherwise, increment pc by length of instuction
      PC = (PC + opcount + 1) & 0xffff;
   }
}

static int em_65816_disassemble(char *buffer, instruction_t *instruction) {

   int numchars;
   int offset;
   char target[16];

   // Unpack the instruction bytes
   int opcode  = instruction->opcode;
   int op1     = instruction->op1;
   int op2     = instruction->op2;
   int op3     = instruction->op3;
   int pc      = instruction->pc;
   int opcount = instruction->opcount;
   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   const char *mnemonic = instr->mnemonic;
   const char *fmt = instr->fmt;
   switch (instr->mode) {
   case IMP:
   case IMPA:
      numchars = sprintf(buffer, fmt, mnemonic);
      break;
   case BRA:
      // Calculate branch target using op1 for normal branches
      offset = (int8_t) op1;
      if (pc < 0) {
         if (offset < 0) {
            sprintf(target, "pc-%d", -offset);
         } else {
            sprintf(target,"pc+%d", offset);
         }
      } else {
         sprintf(target, "%04X", (pc + 2 + offset) & 0xffff);
      }
      numchars = sprintf(buffer, fmt, mnemonic, target);
      break;
   case BRL:
      // Calculate branch target using op1 for normal branches
      offset = (int16_t) ((op2 << 8) + op1);
      if (pc < 0) {
         if (offset < 0) {
            sprintf(target, "pc-%d", -offset);
         } else {
            sprintf(target,"pc+%d", offset);
         }
      } else {
         sprintf(target, "%04X", (pc + 3 + offset) & 0xffff);
      }
      numchars = sprintf(buffer, fmt, mnemonic, target);
      break;
   case ZPR:
      // Calculate branch target using op2 for BBR/BBS
      offset = (int8_t) op2;
      if (pc < 0) {
         if (offset < 0) {
            sprintf(target, "pc-%d", -offset);
         } else {
            sprintf(target,"pc+%d", offset);
         }
      } else {
         sprintf(target, "%04X", (pc + 3 + offset) & 0xffff);
      }
      numchars = sprintf(buffer, fmt, mnemonic, op1, target);
      break;
   case IMM:
      if (opcount == 2) {
         numchars = sprintf(buffer, fmt_imm16, mnemonic, op1, op2);
      } else {
         numchars = sprintf(buffer, fmt, mnemonic, op1);
      }
      break;
   case ZP:
   case ZPX:
   case ZPY:
   case INDX:
   case INDY:
   case IND:
   case SR:
   case ISY:
   case IDL:
   case IDLY:
      numchars = sprintf(buffer, fmt, mnemonic, op1);
      break;
   case ABS:
   case ABSX:
   case ABSY:
   case IND16:
   case IND1X:
   case IAL:
   case BM:
      numchars = sprintf(buffer, fmt, mnemonic, op1, op2);
      break;
   case ABL:
   case ALX:
      numchars = sprintf(buffer, fmt, mnemonic, op1, op2, op3);
      break;
   default:
      numchars = 0;
   }

   return numchars;
}

static int em_65816_get_PC() {
   return PC;
}

static int em_65816_get_PB() {
   return PB;
}

static int em_65816_read_memory(int address) {
   return memory[address];
}

static char *em_65816_get_state(char *buffer) {
   strcpy(buffer, default_state);
   if (B >= 0) {
      write_hex2(buffer + OFFSET_B, B);
   }
   if (A >= 0) {
      write_hex2(buffer + OFFSET_A, A);
   }
   if (X >= 0) {
      write_hex4(buffer + OFFSET_X, X);
   }
   if (Y >= 0) {
      write_hex4(buffer + OFFSET_Y, Y);
   }
   if (SH >= 0) {
      write_hex2(buffer + OFFSET_SH, SH);
   }
   if (SL >= 0) {
      write_hex2(buffer + OFFSET_SL, SL);
   }
   if (N >= 0) {
      buffer[OFFSET_N] = '0' + N;
   }
   if (V >= 0) {
      buffer[OFFSET_V] = '0' + V;
   }
   if (MS >= 0) {
      buffer[OFFSET_MS] = '0' + MS;
   }
   if (XS >= 0) {
      buffer[OFFSET_XS] = '0' + XS;
   }
   if (D >= 0) {
      buffer[OFFSET_D] = '0' + D;
   }
   if (I >= 0) {
      buffer[OFFSET_I] = '0' + I;
   }
   if (Z >= 0) {
      buffer[OFFSET_Z] = '0' + Z;
   }
   if (C >= 0) {
      buffer[OFFSET_C] = '0' + C;
   }
   if (E >= 0) {
      buffer[OFFSET_E] = '0' + E;
   }
   if (PB >= 0) {
      write_hex2(buffer + OFFSET_PB, PB);
   }
   if (DB >= 0) {
      write_hex2(buffer + OFFSET_DB, DB);
   }
   if (DP >= 0) {
      write_hex4(buffer + OFFSET_DP, DP);
   }
   return buffer + OFFSET_END;
}

static int em_65816_get_and_clear_fail() {
   int ret = failflag;
   failflag = 0;
   return ret;
}

cpu_emulator_t em_65816 = {
   .init = em_65816_init,
   .match_interrupt = em_65816_match_interrupt,
   .count_cycles = em_65816_count_cycles,
   .reset = em_65816_reset,
   .interrupt = em_65816_interrupt,
   .emulate = em_65816_emulate,
   .disassemble = em_65816_disassemble,
   .get_PC = em_65816_get_PC,
   .get_PB = em_65816_get_PB,
   .read_memory = em_65816_read_memory,
   .get_state = em_65816_get_state,
   .get_and_clear_fail = em_65816_get_and_clear_fail,
};

// ====================================================================
// 65816 specific instructions
// ====================================================================

// Push Effective Absolute Address
static int op_PEA(operand_t operand, ea_t ea) {
   // always pushes a 16-bit value
   push16(ea);
   return -1;
}

// Push Effective Relative Address
static int op_PER(operand_t operand, ea_t ea) {
   // always pushes a 16-bit value
   push16(ea);
   return -1;
}

// Push Effective Indirect Address
static int op_PEI(operand_t operand, ea_t ea) {
   // always pushes a 16-bit value
   push16(operand);
   return -1;
}

// Push Data Bank Register
static int op_PHB(operand_t operand, ea_t ea) {
   push8(operand);
   if (DB >= 0) {
      if (operand != DB) {
         failflag = 1;
      }
   }
   DB = operand;
   return -1;
}

// Push Program Bank Register
static int op_PHK(operand_t operand, ea_t ea) {
   push8(operand);
   if (PB >= 0) {
      if (operand != PB) {
         failflag = 1;
      }
   }
   PB = operand;
   return -1;
}

// Push Direct Page Register
static int op_PHD(operand_t operand, ea_t ea) {
   push16(operand);
   if (DP >= 0) {
      if (operand != DP) {
         failflag = 1;
      }
   }
   DP = operand;
   return -1;
}

// Pull Data Bank Register
static int op_PLB(operand_t operand, ea_t ea) {
   DB = operand;
   set_NZ8(DB);
   pop8(operand);
   return -1;
}

// Pull Direct Page Register
static int op_PLD(operand_t operand, ea_t ea) {
   DP = operand;
   set_NZ16(DP);
   pop16(operand);
   return -1;
}


// Block Move (Decrementing)
static int op_MVP(operand_t operand, ea_t ea) {
   if (A >= 0 && B >= 0) {
      int C = (((B << 8) | A) - 1) & 0xffff;
      A = C & 0xff;
      B = (C >> 8) & 0xff;
      if (X >= 0) {
         X = (X - 1) & 0xFFFF;
      }
      if (Y >= 0) {
         Y = (Y - 1) & 0xFFFF;
      }
      if (PC >= 0 && C != 0xFFFF) {
         PC -= 3;
      }
   } else {
      A = -1;
      B = -1;
      X = -1;
      Y = -1;
      PC = -1;
   }
   return -1;
}

// Block Move (Incrementing)
static int op_MVN(operand_t operand, ea_t ea) {
   if (A >= 0 && B >= 0) {
      int C = (((B << 8) | A) - 1) & 0xffff;
      A = C & 0xff;
      B = (C >> 8) & 0xff;
      if (X >= 0) {
         X = (X + 1) & 0xFFFF;
      }
      if (Y >= 0) {
         Y = (Y + 1) & 0xFFFF;
      }
      if (PC >= 0 && C != 0xFFFF) {
         PC -= 3;
      }
   } else {
      A = -1;
      B = -1;
      X = -1;
      Y = -1;
      PC = -1;
   }
   return -1;
}

// Transfer Transfer C accumulator to Direct Page register
static int op_TCD(operand_t operand, ea_t ea) {
   // Always a 16-bit transfer
   if (B >= 0 && A >= 0) {
      DP = (B << 8) + A;
      set_NZ16(DP);
   } else {
      DP = -1;
      set_NZ_unknown();
   }
   return -1;
}

// Transfer Transfer C accumulator to Stack pointer
static int op_TCS(operand_t operand, ea_t ea) {
   SH = B;
   SL = A;
   return -1;
}

// Transfer Transfer Direct Page register to C accumulator
static int op_TDC(operand_t operand, ea_t ea) {
   // Always a 16-bit transfer
   if (DP >= 0) {
      A = DP & 255;
      B = (DP >> 8) & 255;
      set_NZ16(DP);
   } else {
      A = -1;
      B = -1;
      set_NZ_unknown();
   }
   return -1;
}

// Transfer Transfer Stack pointer to C accumulator
static int op_TSC(operand_t operand, ea_t ea) {
   // Always a 16-bit transfer
   A = SL;
   B = SH;
   if (B >= 0 && A >= 0) {
      set_NZ16((B << 8) + A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_TXY(operand_t operand, ea_t ea) {
   // Variable size transfer controlled by XS
   if (X >= 0) {
      Y = X;
      set_NZ_XS(Y);
   } else {
      Y = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_TYX(operand_t operand, ea_t ea) {
   // Variable size transfer controlled by XS
   if (Y >= 0) {
      X = Y;
      set_NZ_XS(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
   return -1;
}

// Exchange A and B
static int op_XBA(operand_t operand, ea_t ea) {
   int tmp = A;
   A = B;
   B = tmp;
   if (A >= 0) {
      // Always based on the 8-bit result of A
      set_NZ8(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_XCE(operand_t operand, ea_t ea) {
   int tmp = C;
   C = E;
   E = tmp;
   if (tmp < 0) {
      MS = -1;
      XS = -1;
      E = -1;
   } else if (tmp > 0) {
      emulation_mode_on();
   } else {
      emulation_mode_off();
   }
   return -1;
}

static void repsep(int operand, int val) {
   if (operand & 0x80) {
      N = val;
   }
   if (operand & 0x40) {
      V = val;
   }
   if (E == 0) {
      if (operand & 0x20) {
         MS = val;
      }
      if (operand & 0x10) {
         XS = val;
      }
   }
   if (operand & 0x08) {
      D = val;
   }
   if (operand & 0x04) {
      I = val;
   }
   if (operand & 0x02) {
      Z = val;
   }
   if (operand & 0x01) {
      C = val;
   }
}

// Reset/Set Processor Status Bits
static int op_REP(operand_t operand, ea_t ea) {
   repsep(operand, 0);
   return -1;
}

static int op_SEP(operand_t operand, ea_t ea) {
   repsep(operand, 1);
   return -1;
}

// Jump to Subroutine Long
static int op_JSL(operand_t operand, ea_t ea) {
   // JSR: the operand is the data pushed to the stack (PB, PCH, PCL)
   push8(operand >> 16); // PB
   push16(operand);      // PC
   return -1;
}

// Return from Subroutine Long
static int op_RTL(operand_t operand, ea_t ea) {
   // RTL: the operand is the data pulled from the stack (PCL, PCH, PB)
   pop16(operand);      // PC
   pop8(operand >> 16); // PB
   // The +1 is handled elsewhere
   PC = operand & 0xffff;
   PB = (operand >> 16) & 0xff;
   return -1;
}

// ====================================================================
// 65816/6502 instructions
// ====================================================================

static int op_ADC(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode ADC - works like a 65C02
         int al;
         int ah;
         ah = 0;
         al = (A & 0xF) + (operand & 0xF) + (C ? 1 : 0);
         if (al > 9) {
            al -= 10;
            al &= 0xF;
            ah = 1;
         }
         ah += ((A >> 4) + (operand >> 4));
         V = (((ah << 4) ^ A) & 128) && !((A ^ operand) & 128);
         C = 0;
         if (ah > 9) {
            C = 1;
            ah -= 10;
            ah &= 0xF;
         }
         A = (al & 0xF) | (ah << 4);
      } else {
         // Normal mode ADC
         int tmp = A + operand + C;
         C = (tmp >> 8) & 1;
         V = (((A ^ operand) & 0x80) == 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
      }
      set_NZ_MS(A);
   } else {
      A = -1;
      set_NVZC_unknown();
   }
   return -1;
}

static int op_AND(operand_t operand, ea_t ea) {
   // A is always updated, regardless of the size
   if (A >= 0) {
      A = A & operand;
   }
   // B is updated only of the size is 16
   if (B >= 0) {
      if (MS == 0) {
         B = B & (operand >> 8);
      } else if (MS < 0) {
         B = -1;
      }
   }
   // Updating NZ is complex, depending on the whether A and/or B are unknown
   set_NZ_AB(A, B);
   return -1;
}

static int op_ASLA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      C = (A >> 7) & 1;
      A = (A << 1) & 255;
      set_NZ_MS(A);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_ASL(operand_t operand, ea_t ea) {
   // In 8-bit mode the uppwe byte is ignored by the memory write code
   int tmp = (operand << 1) & 0xffff;
   if (MS > 0) {
      // 8-bit mode
      C = (operand >> 7) & 1;
      tmp = (operand << 1) & 0xff;
   } else if (MS == 0) {
      // 16-bit mode
      C = (operand >> 15) & 1;
   } else {
      // mode unknown
      C = -1;
   }
   set_NZ_MS(tmp);
   return tmp;
}

static int op_BCC(operand_t branch_taken, ea_t ea) {
   if (C >= 0) {
      if (C == branch_taken) {
         failflag = 1;
      }
   } else {
      C = 1 - branch_taken;
   }
   return -1;
}

static int op_BCS(operand_t branch_taken, ea_t ea) {
   if (C >= 0) {
      if (C != branch_taken) {
         failflag = 1;
      }
   } else {
      C = branch_taken;
   }
   return -1;
}

static int op_BNE(operand_t branch_taken, ea_t ea) {
   if (Z >= 0) {
      if (Z == branch_taken) {
         failflag = 1;
      }
   } else {
      Z = 1 - branch_taken;
   }
   return -1;
}

static int op_BEQ(operand_t branch_taken, ea_t ea) {
   if (Z >= 0) {
      if (Z != branch_taken) {
         failflag = 1;
        }
   } else {
      Z = branch_taken;
   }
   return -1;
}

static int op_BPL(operand_t branch_taken, ea_t ea) {
   if (N >= 0) {
      if (N == branch_taken) {
         failflag = 1;
      }
   } else {
      N = 1 - branch_taken;
   }
   return -1;
}

static int op_BMI(operand_t branch_taken, ea_t ea) {
   if (N >= 0) {
      if (N != branch_taken) {
         failflag = 1;
      }
   } else {
      N = branch_taken;
   }
   return -1;
}

static int op_BVC(operand_t branch_taken, ea_t ea) {
   if (V >= 0) {
      if (V == branch_taken) {
         failflag = 1;
      }
   } else {
      V = 1 - branch_taken;
   }
   return -1;
}

static int op_BVS(operand_t branch_taken, ea_t ea) {
   if (V >= 0) {
      if (V != branch_taken) {
         failflag = 1;
        }
   } else {
      V = branch_taken;
   }
   return -1;
}

static int op_BIT_IMM(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
   return -1;
}

static int op_BIT(operand_t operand, ea_t ea) {
   if (MS > 0) {
      // 8-bit mode
      N = (operand >> 7) & 1;
      V = (operand >> 6) & 1;
   } else if (MS == 0) {
      // 16-bit mode
      N = (operand >> 15) & 1;
      V = (operand >> 14) & 1;
   } else {
      // mode undefined
      N = -1; // could be less pessimistic
      V = -1; // could be less pessimistic
   }
   if (operand == 0) {
      // This makes the remainder less pessimistic
      Z = 1;
   } else  if (MS > 0) {
      // 8-bit mode
      Z = (A & operand) == 0;
   } else if (MS == 0 && A >= 0 && B >= 0) {
      // 16-bit mode
      Z = (((B << 8) + A) & operand) == 0;
   } else {
      // mode undefined
      Z = -1;
   }
   return -1;
}

static int op_CLC(operand_t operand, ea_t ea) {
   C = 0;
   return -1;
}

static int op_CLD(operand_t operand, ea_t ea) {
   D = 0;
   return -1;
}

static int op_CLI(operand_t operand, ea_t ea) {
   I = 0;
   return -1;
}

static int op_CLV(operand_t operand, ea_t ea) {
   V = 0;
   return -1;
}

static int op_CMP(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      int tmp = A - operand;
      C = tmp >= 0;
      set_NZ_MS(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_CPX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      int tmp = X - operand;
      C = tmp >= 0;
      set_NZ_XS(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_CPY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      int tmp = Y - operand;
      C = tmp >= 0;
      set_NZ_XS(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_DECA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      A = (A - 1) & 255;
      set_NZ_MS(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_DEC(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   int tmp = (operand - 1) & 255;
   set_NZ_MS(tmp);
   return tmp;
}

static int op_DEX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      if (XS > 0) {
         // 8-bit mode
         X = (X - 1) & 0xff;
         set_NZ8(X);
      } else if (XS == 0) {
         // 16-bit mode
         X = (X - 1) & 0xffff;
         set_NZ16(X);
      } else {
         // mode undefined
         X = -1;
         set_NZ_unknown();
      }
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_DEY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      if (XS > 0) {
         // 8-bit mode
         Y = (Y - 1) & 0xff;
         set_NZ8(Y);
      } else if (XS == 0) {
         // 16-bit mode
         Y = (Y - 1) & 0xffff;
         set_NZ16(Y);
      } else {
         // mode undefined
         Y = -1;
         set_NZ_unknown();
      }
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_EOR(operand_t operand, ea_t ea) {
   // A is always updated, regardless of the size
   if (A >= 0) {
      A = A ^ operand;
   }
   // B is updated only of the size is 16
   if (B >= 0) {
      if (MS == 0) {
         B = B ^ (operand >> 8);
      } else if (MS < 0) {
         B = -1;
      }
   }
   // Updating NZ is complex, depending on the whether A and/or B are unknown
   set_NZ_AB(A, B);
   return -1;
}

static int op_INCA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      A = (A + 1) & 255;
      set_NZ_MS(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_INC(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   int tmp = (operand + 1) & 255;
   set_NZ_MS(tmp);
   return tmp;
}

static int op_INX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      if (XS > 0) {
         // 8-bit mode
         X = (X + 1) & 0xff;
         set_NZ8(X);
      } else if (XS == 0) {
         // 16-bit mode
         X = (X + 1) & 0xffff;
         set_NZ16(X);
      } else {
         // mode undefined
         X = -1;
         set_NZ_unknown();
      }
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_INY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      if (XS > 0) {
         // 8-bit mode
         Y = (Y + 1) & 0xff;
         set_NZ8(Y);
      } else if (XS == 0) {
         // 16-bit mode
         Y = (Y + 1) & 0xffff;
         set_NZ16(Y);
      } else {
         // mode undefined
         Y = -1;
         set_NZ_unknown();
      }
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_JSR(operand_t operand, ea_t ea) {
   // JSR: the operand is the data pushed to the stack (PCH, PCL)
   push16(operand);  // PC
   return -1;
}

static int op_LDA(operand_t operand, ea_t ea) {
   A = operand;
   if (!MS) {
      B = (operand >> 8) & 0xff;
   }
   set_NZ_MS(A);
   return -1;
}

static int op_LDX(operand_t operand, ea_t ea) {
   X = operand;
   set_NZ_XS(X);
   return -1;
}

static int op_LDY(operand_t operand, ea_t ea) {
   Y = operand;
   set_NZ_XS(Y);
   return -1;
}

static int op_LSRA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0) {
      C = A & 1;
      A = A >> 1;
      set_NZ_MS(A);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_LSR(operand_t operand, ea_t ea) {
   // In 8-bit mode the uppwe byte is ignored by the memory write code
   C = operand & 1;
   int tmp = operand >> 1;
   set_NZ_MS(tmp);
   return tmp;
}

static int op_ORA(operand_t operand, ea_t ea) {
   // A is always updated, regardless of the size
   if (A >= 0) {
      A = A | operand;
   }
   // B is updated only of the size is 16
   if (B >= 0) {
      if (MS == 0) {
         B = B | (operand >> 8);
      } else if (MS < 0) {
         B = -1;
      }
   }
   // Updating NZ is complex, depending on the whether A and/or B are unknown
   set_NZ_AB(A, B);
   return -1;
}

static int op_PHA(operand_t operand, ea_t ea) {
   pushMS(operand);
   op_STA(operand, -1);
   return -1;
}

static int op_PHP(operand_t operand, ea_t ea) {
   push8(operand);
   check_FLAGS(operand);
   set_FLAGS(operand);
   return -1;
}

static int op_PHX(operand_t operand, ea_t ea) {
   pushXS(operand);
   op_STX(operand, -1);
   return -1;
}

static int op_PHY(operand_t operand, ea_t ea) {
   pushXS(operand);
   op_STY(operand, -1);
   return -1;
}

static int op_PLA(operand_t operand, ea_t ea) {
   A = operand & 0xff;
   if (MS < 0) {
      B = -1;
   } else if (MS == 0) {
      B = (operand >> 8);
   }
   set_NZ_MS(operand);
   popMS(operand);
   return -1;
}

static int op_PLP(operand_t operand, ea_t ea) {
   set_FLAGS(operand);
   pop8(operand);
   return -1;
}

static int op_PLX(operand_t operand, ea_t ea) {
   X = operand;
   set_NZ_XS(X);
   popXS(operand);
   return -1;
}

static int op_PLY(operand_t operand, ea_t ea) {
   Y = operand;
   set_NZ_XS(Y);
   popXS(operand);
   return -1;
}

static int op_ROLA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0 && C >= 0) {
      int tmp = (A << 1) + C;
      C = (tmp >> 8) & 1;
      A = tmp & 255;
      set_NZ_MS(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
   return -1;
}

static int op_ROL(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (C >= 0) {
      int tmp = (operand << 1) + C;
      C = (tmp >> 8) & 1;
      tmp = tmp & 255;
      set_NZ_MS(tmp);
      memory_write(tmp, ea);
      return tmp;
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_RORA(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0 && C >= 0) {
      int tmp = (A >> 1) + (C << 7);
      C = A & 1;
      A = tmp;
      set_NZ_MS(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
   return -1;
}

static int op_ROR(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (C >= 0) {
      int tmp = (operand >> 1) + (C << 7);
      C = operand & 1;
      set_NZ_MS(tmp);
      return tmp;
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_RTS(operand_t operand, ea_t ea) {
   // RTS: the operand is the data pulled from the stack (PCL, PCH)
   pop8(operand);
   pop8(operand >> 8);
   // The +1 is handled elsewhere
   PC = operand & 0xffff;
   return -1;
}

static int op_RTI(operand_t operand, ea_t ea) {
   // RTI: the operand is the data pulled from the stack (P, PCL, PCH, PBR)
   set_FLAGS(operand);
   pop8(operand);
   pop8(operand >> 8);
   pop8(operand >> 16);
   if (E == 0) {
      pop8(operand >> 24);
   }
   return -1;
}

static int op_SBC(operand_t operand, ea_t ea) {
   // TODO: Make variable size
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode SBC - works like a 65C02
         int al;
         int tmp;
         // On 65C02 SBC, both flags and A can be different to the 6502
         al = (A & 15) - (operand & 15) - ((C) ? 0 : 1);
         tmp = A - operand - ((C) ? 0 : 1);
         C = (tmp & 0x100) ? 0 : 1;
         V = ((A ^ operand) & 0x80) && ((A ^ tmp) & 0x80);
         if (tmp < 0) {
            tmp = tmp - 0x60;
         }
         if (al < 0) {
            tmp = tmp - 0x06;
         }
         A = tmp & 255;
      } else {
         // Normal mode SBC
         int tmp = A - operand - (1 - C);
         C = 1 - ((tmp >> 8) & 1);
         V = (((A ^ operand) & 0x80) != 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
      }
      set_NZ_MS(A);
   } else {
      A = -1;
      set_NVZC_unknown();
   }
   return -1;
}

static int op_SEC(operand_t operand, ea_t ea) {
   C = 1;
   return -1;
}

static int op_SED(operand_t operand, ea_t ea) {
   D = 1;
   return -1;
}

static int op_SEI(operand_t operand, ea_t ea) {
   I = 1;
   return -1;
}

static int op_STA(operand_t operand, ea_t ea) {
   int oplo = operand & 0xff;
   int ophi = (operand >> 8) & 0xff;
   // Always write A
   if (A >= 0) {
      if (oplo != A) {
         failflag = 1;
      }
   }
   A = oplo;
   // Optionally write B, depending on the MS flag
   if (MS < 0) {
      B = -1;
   } else if (MS == 0) {
      if (B >= 0) {
         if (ophi != B) {
            failflag = 1;
         }
      }
      B = ophi;
   }
   return operand;
}

static int op_STX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      if (operand != X) {
         failflag = 1;
      }
   }
   X = operand;
   return operand;
}

static int op_STY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      if (operand != Y) {
         failflag = 1;
      }
   }
   Y = operand;
   return operand;
}

static int op_STZ(operand_t operand, ea_t ea) {
   if (operand != 0) {
      failflag = 1;
   }
   return operand;
}

static int op_TSB(operand_t operand, ea_t ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         return memory[ea] | A;
      }
   } else {
      Z = -1;
   }
   return -1;
}
static int op_TRB(operand_t operand, ea_t ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         return (memory[ea] & ~A);
      }
   } else {
      Z = -1;
   }
   return -1;
}

// This is used to implement: TAX, TAY, TSX
static void transfer_88_16(int srchi, int srclo, int *dst) {
   if (srclo >= 0 && srchi >=0 && XS == 0) {
      // 16-bit
      *dst = (srchi << 8) + srclo;
      set_NZ16(*dst);
   } else if (srclo >= 0 && XS == 1) {
      // 8-bit
      *dst = srclo;
      set_NZ8(*dst);
   } else {
      *dst = -1;
      set_NZ_unknown();
   }
}

// This is used to implement: TXA, TYA
static void transfer_16_88(int src, int *dsthi, int *dstlo) {
   if (MS == 0) {
      // 16-bit
      if (src >= 0) {
         *dsthi = (src >> 8) & 0xff;
         *dstlo = src & 0xff;
         set_NZ16(src);
      } else {
         *dsthi = -1;
         *dstlo = -1;
         set_NZ_unknown();
      }
   } if (MS == 1) {
      // 8-bit
      if (src >= 0) {
         *dstlo = src & 0xff;
         set_NZ8(src);
      } else {
         *dstlo = -1;
         set_NZ_unknown();
      }
   } else {
      // MS undefined
      if (src >= 0) {
         *dstlo = src & 0xff;
      } else {
         *dstlo = -1;
      }
      *dsthi = -1;
      set_NZ_unknown();
   }
}

static int op_TAX(operand_t operand, ea_t ea) {
   transfer_88_16(B, A, &X);
   return -1;
}

static int op_TAY(operand_t operand, ea_t ea) {
   transfer_88_16(B, A, &Y);
   return -1;
}

static int op_TSX(operand_t operand, ea_t ea) {
   transfer_88_16(SH, SL, &X);
   return -1;
}

static int op_TXA(operand_t operand, ea_t ea) {
   transfer_16_88(X, &B, &A);
   return -1;
}

static int op_TXS(operand_t operand, ea_t ea) {
   if (X >= 0) {
      SH = (X >> 8) & 0xff;
      SL = (X     ) & 0xff;
   } else {
      SH = -1;
      SL = -1;
   }
   // Force SH to be 01 in emulation mode
   if (E == 1) {
      SH = 0x01;
   }
   return -1;
}

static int op_TYA(operand_t operand, ea_t ea) {
   transfer_16_88(Y, &B, &A);
   return -1;
}

// ====================================================================
// Opcode Tables
// ====================================================================

static InstrType instr_table_65c816[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, OTHER,    0},
   /* 01 */   { "ORA",  0, INDX  , 6, READOP,   op_ORA},
   /* 02 */   { "COP",  0, IMM   , 7, OTHER,    0},
   /* 03 */   { "ORA",  0, SR    , 4, READOP,   op_ORA},
   /* 04 */   { "TSB",  0, ZP    , 5, TSBTRBOP, op_TSB},
   /* 05 */   { "ORA",  0, ZP    , 3, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, RMWOP,    op_ASL},
   /* 07 */   { "ORA",  0, IDL   , 6, READOP,   op_ORA},
   /* 08 */   { "PHP",  0, IMP   , 3, OTHER,    op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, OTHER,    op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, OTHER,    op_ASLA},
   /* 0B */   { "PHD",  0, IMP   , 4, OTHER,    op_PHD},
   /* 0C */   { "TSB",  0, ABS   , 6, TSBTRBOP, op_TSB},
   /* 0D */   { "ORA",  0, ABS   , 4, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, RMWOP,    op_ASL},
   /* 0F */   { "ORA",  0, ABL   , 5, READOP,   op_ORA},
   /* 10 */   { "BPL",  0, BRA   , 2, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, READOP,   op_ORA},
   /* 12 */   { "ORA",  0, IND   , 5, READOP,   op_ORA},
   /* 13 */   { "ORA",  0, ISY   , 7, READOP,   op_ORA},
   /* 14 */   { "TRB",  0, ZP    , 5, TSBTRBOP, op_TRB},
   /* 15 */   { "ORA",  0, ZPX   , 4, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, RMWOP,    op_ASL},
   /* 17 */   { "ORA",  0, IDLY  , 6, READOP,   op_ORA},
   /* 18 */   { "CLC",  0, IMP   , 2, OTHER,    op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, READOP,   op_ORA},
   /* 1A */   { "INC",  0, IMPA  , 2, OTHER,    op_INCA},
   /* 1B */   { "TCS",  0, IMP   , 2, OTHER,    op_TCS},
   /* 1C */   { "TRB",  0, ABS   , 6, TSBTRBOP, op_TRB},
   /* 1D */   { "ORA",  0, ABSX  , 4, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 7, RMWOP,    op_ASL},
   /* 1F */   { "ORA",  0, ALX   , 5, READOP,   op_ORA},
   /* 20 */   { "JSR",  0, ABS   , 6, OTHER,    op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, READOP,   op_AND},
   /* 22 */   { "JSL",  0, ABL   , 8, OTHER,    op_JSL},
   /* 23 */   { "AND",  0, SR    , 4, READOP,   op_AND},
   /* 24 */   { "BIT",  0, ZP    , 3, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, RMWOP,    op_ROL},
   /* 27 */   { "AND",  0, IDL   , 6, READOP,   op_AND},
   /* 28 */   { "PLP",  0, IMP   , 4, OTHER,    op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, OTHER,    op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, OTHER,    op_ROLA},
   /* 2B */   { "PLD",  0, IMP   , 5, OTHER,    op_PLD},
   /* 2C */   { "BIT",  0, ABS   , 4, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, RMWOP,    op_ROL},
   /* 2F */   { "AND",  0, ABL   , 5, READOP,   op_AND},
   /* 30 */   { "BMI",  0, BRA   , 2, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, READOP,   op_AND},
   /* 32 */   { "AND",  0, IND   , 5, READOP,   op_AND},
   /* 33 */   { "AND",  0, ISY   , 7, READOP,   op_AND},
   /* 34 */   { "BIT",  0, ZPX   , 4, READOP,   op_BIT},
   /* 35 */   { "AND",  0, ZPX   , 4, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, RMWOP,    op_ROL},
   /* 37 */   { "AND",  0, IDLY  , 6, READOP,   op_AND},
   /* 38 */   { "SEC",  0, IMP   , 2, OTHER,    op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, READOP,   op_AND},
   /* 3A */   { "DEC",  0, IMPA  , 2, OTHER,    op_DECA},
   /* 3B */   { "TSC",  0, IMP   , 2, OTHER,    op_TSC},
   /* 3C */   { "BIT",  0, ABSX  , 4, READOP,   op_BIT},
   /* 3D */   { "AND",  0, ABSX  , 4, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 7, RMWOP,    op_ROL},
   /* 3F */   { "AND",  0, ALX   , 5, READOP,   op_AND},
   /* 40 */   { "RTI",  0, IMP   , 6, OTHER,    op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, READOP,   op_EOR},
   /* 42 */   { "WDM",  0, IMM   , 2, OTHER,    0},
   /* 43 */   { "EOR",  0, SR    , 4, READOP,   op_EOR},
   /* 44 */   { "MVP",  0, BM    , 7, OTHER,    op_MVP}, // TODO: Memory Modelling
   /* 45 */   { "EOR",  0, ZP    , 3, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, RMWOP,    op_LSR},
   /* 47 */   { "EOR",  0, IDL   , 6, READOP,   op_EOR},
   /* 48 */   { "PHA",  0, IMP   , 3, OTHER,    op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, OTHER,    op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, OTHER,    op_LSRA},
   /* 4B */   { "PHK",  0, IMP   , 3, OTHER,    op_PHK},
   /* 4C */   { "JMP",  0, ABS   , 3, OTHER,    0},
   /* 4D */   { "EOR",  0, ABS   , 4, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, RMWOP,    op_LSR},
   /* 4F */   { "EOR",  0, ABL   , 5, READOP,   op_EOR},
   /* 50 */   { "BVC",  0, BRA   , 2, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, READOP,   op_EOR},
   /* 52 */   { "EOR",  0, IND   , 5, READOP,   op_EOR},
   /* 53 */   { "EOR",  0, ISY   , 7, READOP,   op_EOR},
   /* 54 */   { "MVN",  0, BM    , 7, OTHER,    op_MVN}, // TODO: Memory Modelling
   /* 55 */   { "EOR",  0, ZPX   , 4, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, RMWOP,    op_LSR},
   /* 57 */   { "EOR",  0, IDLY  , 6, READOP,   op_EOR},
   /* 58 */   { "CLI",  0, IMP   , 2, OTHER,    op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, READOP,   op_EOR},
   /* 5A */   { "PHY",  0, IMP   , 3, OTHER,    op_PHY},
   /* 5B */   { "TCD",  0, IMP   , 2, OTHER,    op_TCD},
   /* 5C */   { "JML",  0, ABL   , 4, OTHER,    0},
   /* 5D */   { "EOR",  0, ABSX  , 4, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 7, RMWOP,    op_LSR},
   /* 5F */   { "EOR",  0, ALX   , 5, READOP,   op_EOR},
   /* 60 */   { "RTS",  0, IMP   , 6, OTHER,    op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, READOP,   op_ADC},
   /* 62 */   { "PER",  0, BRL   , 6, OTHER,    op_PER},
   /* 63 */   { "ADC",  0, SR    , 4, READOP,   op_ADC},
   /* 64 */   { "STZ",  0, ZP    , 3, WRITEOP,  op_STZ},
   /* 65 */   { "ADC",  0, ZP    , 3, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, RMWOP,    op_ROR},
   /* 67 */   { "ADC",  0, IDL   , 6, READOP,   op_ADC},
   /* 68 */   { "PLA",  0, IMP   , 4, OTHER,    op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, OTHER,    op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, OTHER,    op_RORA},
   /* 6B */   { "RTL",  0, IMP   , 6, OTHER,    op_RTL},
   /* 6C */   { "JMP",  0, IND16 , 5, OTHER,    0},
   /* 6D */   { "ADC",  0, ABS   , 4, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, RMWOP,    op_ROR},
   /* 6F */   { "ADC",  0, ABL   , 5, READOP,   op_ADC},
   /* 70 */   { "BVS",  0, BRA   , 2, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, READOP,   op_ADC},
   /* 72 */   { "ADC",  0, IND   , 5, READOP,   op_ADC},
   /* 73 */   { "ADC",  0, ISY   , 7, READOP,   op_ADC},
   /* 74 */   { "STZ",  0, ZPX   , 4, WRITEOP,  op_STZ},
   /* 75 */   { "ADC",  0, ZPX   , 4, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, RMWOP,    op_ROR},
   /* 77 */   { "ADC",  0, IDLY  , 6, READOP,   op_ADC},
   /* 78 */   { "SEI",  0, IMP   , 2, OTHER,    op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, READOP,   op_ADC},
   /* 7A */   { "PLY",  0, IMP   , 4, OTHER,    op_PLY},
   /* 7B */   { "TDC",  0, IMP   , 2, OTHER,    op_TDC},
   /* 7C */   { "JMP",  0, IND1X , 6, OTHER,    0},
   /* 7D */   { "ADC",  0, ABSX  , 4, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 7, RMWOP,    op_ROR},
   /* 7F */   { "ADC",  0, ALX   , 5, READOP,   op_ADC},
   /* 80 */   { "BRA",  0, BRA   , 3, OTHER,    0},
   /* 81 */   { "STA",  0, INDX  , 6, WRITEOP,  op_STA},
   /* 82 */   { "BRL",  0, BRL   , 4, OTHER,    0},
   /* 83 */   { "STA",  0, SR    , 4, WRITEOP,  op_STA},
   /* 84 */   { "STY",  0, ZP    , 3, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, WRITEOP,  op_STX},
   /* 87 */   { "STA" , 0, IDL   , 6, WRITEOP,  op_STA},
   /* 88 */   { "DEY",  0, IMP   , 2, OTHER,    op_DEY},
   /* 89 */   { "BIT",  0, IMM   , 2, OTHER,    op_BIT_IMM},
   /* 8A */   { "TXA",  0, IMP   , 2, OTHER,    op_TXA},
   /* 8B */   { "PHB",  0, IMP   , 3, OTHER,    op_PHB},
   /* 8C */   { "STY",  0, ABS   , 4, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, WRITEOP,  op_STX},
   /* 8F */   { "STA",  0, ABL   , 5, WRITEOP,  op_STA},
   /* 90 */   { "BCC",  0, BRA   , 2, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, WRITEOP,  op_STA},
   /* 92 */   { "STA",  0, IND   , 5, WRITEOP,  op_STA},
   /* 93 */   { "STA",  0, ISY   , 7, WRITEOP,  op_STA},
   /* 94 */   { "STY",  0, ZPX   , 4, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, WRITEOP,  op_STX},
   /* 97 */   { "STA",  0, IDLY  , 6, WRITEOP,  op_STA},
   /* 98 */   { "TYA",  0, IMP   , 2, OTHER,    op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, OTHER,    op_TXS},
   /* 9B */   { "TXY",  0, IMP   , 2, OTHER,    op_TXY},
   /* 9C */   { "STZ",  0, ABS   , 4, WRITEOP,  op_STZ},
   /* 9D */   { "STA",  0, ABSX  , 5, WRITEOP,  op_STA},
   /* 9E */   { "STZ",  0, ABSX  , 5, WRITEOP,  op_STZ},
   /* 9F */   { "STA",  0, ALX   , 5, WRITEOP,  op_STA},
   /* A0 */   { "LDY",  0, IMM   , 2, OTHER,    op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, OTHER,    op_LDX},
   /* A3 */   { "LDA",  0, SR    , 4, READOP,   op_LDA},
   /* A4 */   { "LDY",  0, ZP    , 3, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, READOP,   op_LDX},
   /* A7 */   { "LDA",  0, IDL   , 6, READOP,   op_LDA},
   /* A8 */   { "TAY",  0, IMP   , 2, OTHER,    op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, OTHER,    op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, OTHER,    op_TAX},
   /* AB */   { "PLB",  0, IMP   , 4, OTHER,    op_PLB},
   /* AC */   { "LDY",  0, ABS   , 4, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, READOP,   op_LDX},
   /* AF */   { "LDA",  0, ABL   , 5, READOP,   op_LDA},
   /* B0 */   { "BCS",  0, BRA   , 2, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, READOP,   op_LDA},
   /* B2 */   { "LDA",  0, IND   , 5, READOP,   op_LDA},
   /* B3 */   { "LDA",  0, ISY   , 7, READOP,   op_LDA},
   /* B4 */   { "LDY",  0, ZPX   , 4, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, READOP,   op_LDX},
   /* B7 */   { "LDA",  0, IDLY  , 6, READOP,   op_LDA},
   /* B8 */   { "CLV",  0, IMP   , 2, OTHER,    op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, OTHER,    op_TSX},
   /* BB */   { "TYX",  0, IMP   , 2, OTHER,    op_TYX},
   /* BC */   { "LDY",  0, ABSX  , 4, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, READOP,   op_LDX},
   /* BF */   { "LDA",  0, ALX   , 5, READOP,   op_LDA},
   /* C0 */   { "CPY",  0, IMM   , 2, OTHER,    op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, READOP,   op_CMP},
   /* C2 */   { "REP",  0, IMM   , 3, OTHER,    op_REP},
   /* C3 */   { "CMP",  0, SR    , 4, READOP,   op_CMP},
   /* C4 */   { "CPY",  0, ZP    , 3, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, RMWOP,    op_DEC},
   /* C7 */   { "CMP",  0, IDL   , 6, READOP,   op_CMP},
   /* C8 */   { "INY",  0, IMP   , 2, OTHER,    op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, OTHER,    op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, OTHER,    op_DEX},
   /* CB */   { "WAI",  0, IMP   , 1, OTHER,    0},        // WD65C02=3
   /* CC */   { "CPY",  0, ABS   , 4, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, RMWOP,    op_DEC},
   /* CF */   { "CMP",  0, ABL   , 5, READOP,   op_CMP},
   /* D0 */   { "BNE",  0, BRA   , 2, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, READOP,   op_CMP},
   /* D2 */   { "CMP",  0, IND   , 5, READOP,   op_CMP},
   /* D3 */   { "CMP",  0, ISY   , 7, READOP,   op_CMP},
   /* D4 */   { "PEI",  0, IND   , 6, OTHER,    op_PEI},
   /* D5 */   { "CMP",  0, ZPX   , 4, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, RMWOP,    op_DEC},
   /* D7 */   { "CMP" , 0, IDLY  , 6, READOP,   op_CMP},
   /* D8 */   { "CLD",  0, IMP   , 2, OTHER,    op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, READOP,   op_CMP},
   /* DA */   { "PHX",  0, IMP   , 3, OTHER,    op_PHX},
   /* DB */   { "STP",  0, IMP   , 1, OTHER,    0},        // WD65C02=3
   /* DC */   { "JMP",  0, IAL   , 6, OTHER,    0},
   /* DD */   { "CMP",  0, ABSX  , 4, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, RMWOP,    op_DEC},
   /* DF */   { "CMP",  0, ALX   , 5, READOP,   op_CMP},
   /* E0 */   { "CPX",  0, IMM   , 2, OTHER,    op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, READOP,   op_SBC},
   /* E2 */   { "SEP",  0, IMM   , 3, OTHER,    op_SEP},
   /* E3 */   { "SBC",  0, SR    , 4, READOP,   op_SBC},
   /* E4 */   { "CPX",  0, ZP    , 3, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, RMWOP,    op_INC},
   /* E7 */   { "SBC",  0, IDL   , 6, READOP,   op_SBC},
   /* E8 */   { "INX",  0, IMP   , 2, OTHER,    op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, OTHER,    op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, OTHER,    0},
   /* EB */   { "XBA",  0, IMP   , 3, OTHER,    op_XBA},
   /* EC */   { "CPX",  0, ABS   , 4, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, RMWOP,    op_INC},
   /* EF */   { "SBC",  0, ABL   , 5, READOP,   op_SBC},
   /* F0 */   { "BEQ",  0, BRA   , 2, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, READOP,   op_SBC},
   /* F2 */   { "SBC",  0, IND   , 5, READOP,   op_SBC},
   /* F3 */   { "SBC",  0, ISY   , 7, READOP,   op_SBC},
   /* F4 */   { "PEA",  0, ABS   , 5, OTHER,    op_PEA},
   /* F5 */   { "SBC",  0, ZPX   , 4, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, RMWOP,    op_INC},
   /* F7 */   { "SBC",  0, IDLY  , 6, READOP,   op_SBC},
   /* F8 */   { "SED",  0, IMP   , 2, OTHER,    op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, READOP,   op_SBC},
   /* FA */   { "PLX",  0, IMP   , 4, OTHER,    op_PLX},
   /* FB */   { "XCE",  0, IMP   , 2, OTHER,    op_XCE},
   /* FC */   { "JSR",  0, IND1X , 8, OTHER,    op_JSR},
   /* FD */   { "SBC",  0, ABSX  , 4, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, RMWOP,    op_INC},
   /* FF */   { "SBC",  0, ALX   , 5, READOP,   op_SBC}
};
