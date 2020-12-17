#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "memory.h"
#include "tube_decode.h"
#include "em_6502.h"

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
   ZPR
} AddrMode ;

typedef enum {
   READOP,
   WRITEOP,
   RMWOP,
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
   int decimalcorrect;
   OpType optype;
   int (*emulate)(operand_t, ea_t);
   int len;
   const char *fmt;
} InstrType;


// ====================================================================
// Static variables
// ====================================================================

#define OFFSET_A    2
#define OFFSET_X    7
#define OFFSET_Y   12
#define OFFSET_S   18
#define OFFSET_N   23
#define OFFSET_V   27
#define OFFSET_D   31
#define OFFSET_I   35
#define OFFSET_Z   39
#define OFFSET_C   43
#define OFFSET_END 44

static const char default_state[] = "A=?? X=?? Y=?? SP=?? N=? V=? D=? I=? Z=? C=?";

static int rockwell;
static int c02;
static int bbctube;
static int master_nordy;

static InstrType *instr_table;

static AddrModeType addr_mode_table[] = {
   {1,    "%1$s"},                  // IMP
   {1,    "%1$s A"},                // IMPA
   {2,    "%1$s %2$s"},             // BRA
   {2,    "%1$s #%2$02X"},          // IMM
   {2,    "%1$s %2$02X"},           // ZP
   {2,    "%1$s %2$02X,X"},         // ZPX
   {2,    "%1$s %2$02X,Y"},         // ZPY
   {2,    "%1$s (%2$02X,X)"},       // INDX
   {2,    "%1$s (%2$02X),Y"},       // INDY
   {2,    "%1$s (%2$02X)"},         // IND
   {3,    "%1$s %3$02X%2$02X"},     // ABS
   {3,    "%1$s %3$02X%2$02X,X"},   // ABSX
   {3,    "%1$s %3$02X%2$02X,Y"},   // ABSY
   {3,    "%1$s (%3$02X%2$02X)"},   // IND1
   {3,    "%1$s (%3$02X%2$02X,X)"}, // IND1X
   {3,    "%1$s %2$02X,%3$s"}       // ZPR
};

// 6502 registers: -1 means unknown
static int A = -1;
static int X = -1;
static int Y = -1;
static int S = -1;
static int PC = -1;

// 6502 flags: -1 means unknown
static int N = -1;
static int V = -1;
static int D = -1;
static int I = -1;
static int Z = -1;
static int C = -1;

static char ILLEGAL[] = "???";

// ====================================================================
// Forward declarations
// ====================================================================

static InstrType instr_table_6502[];
static InstrType instr_table_65c02[];

static int op_STA(operand_t operand, ea_t ea);
static int op_STX(operand_t operand, ea_t ea);
static int op_STY(operand_t operand, ea_t ea);

// ====================================================================
// Helper Methods
// ====================================================================

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

static void set_NZ(int value) {
   N = (value & 128) > 0;
   Z = value == 0;
}


static void pop8(int value) {
   if (S >= 0) {
      S = (S + 1) & 0xff;
      memory_read(value & 0xff, 0x100 + S, MEM_STACK);
   }
}

static void push8(int value) {
   if (S >= 0) {
      memory_write(value & 0xff, 0x100 + S, MEM_STACK);
      S = (S - 1) & 0xff;
   }
}

static void push16(int value) {
   push8(value >> 8);
   push8(value);
}

static void interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction, int pc_offset) {
   // Parse the bus cycles
   // <opcode> <op1> <write pch> <write pcl> <write p> <read rst> <read rsth>
   int pc     = (sample_q[2].data << 8) + sample_q[3].data;
   int flags  = sample_q[4].data;
   int vector = (sample_q[6].data << 8) + sample_q[5].data;
   // Update the address of the interruted instruction
   instruction->pc = (pc - pc_offset) & 0xffff;
   // Stack the PB/PC/FLags (for memory modelling)
   push16(pc);
   push8(flags);
   // Validate the flags
   check_FLAGS(flags);
   // And make them consistent
   set_FLAGS(flags);
   // Setup expected state for the ISR
   I = 1;
   if (c02) {
      D = 0;
   }
   PC = vector;
}

static int count_cycles_without_sync(sample_t *sample_q, int intr_seen) {

   static int mhz1_phase = 1;

   if (intr_seen) {
      mhz1_phase ^= 1;
      return 7;
   }

   int opcode = sample_q[0].data;
   int op1    = sample_q[1].data;
   int op2    = sample_q[opcode == 0x20 ? 5 : ((opcode & 0x0f) == 0x0f) ? 4 : 2].data;

   InstrType *instr = &instr_table[opcode];

   int cycle_count = instr->cycles;

   // Account for extra cycle in ADC/SBC in decimal mode in C02
   if (c02 && instr->decimalcorrect && D == 1) {
      cycle_count++;
   }

   // Account for extra cycle in a page crossing in (indirect), Y (not stores)
   // <opcpde> <op1> <addrlo> <addrhi> [ <page crossing>] <<operand> [ <extra cycle in dec mode> ]
   if ((instr->mode == INDY) && (instr->optype != WRITEOP) && Y >= 0) {
      int base = (sample_q[3].data << 8) + sample_q[2].data;
      if ((base & 0xff00) != ((base + Y) & 0xff00)) {
         cycle_count++;
      }
   }

   // Account for extra cycle in a page crossing in absolute indexed (not stores)
   if (((instr->mode == ABSX) || (instr->mode == ABSY)) && (instr->optype != WRITEOP)) {
      // 6502:  Need to exclude ASL/ROL/LSR/ROR/DEC/INC, which are 7 cycles regardless
      // 65C02: Need to exclude DEC/INC, which are 7 cycles regardless
      if ((opcode != 0xDE) && (opcode != 0xFE) && (c02 || ((opcode != 0x1E) && (opcode != 0x3E) && (opcode != 0x5E) && (opcode != 0x7E)))) {
         int index = (instr->mode == ABSX) ? X : Y;
         if (index >= 0) {
            int base = op1 + (op2 << 8);
            if ((base & 0xff00) != ((base + index) & 0xff00)) {
               cycle_count++;
            }
         }
      }
   }

   // Account for extra cycles in BBR/BBS
   //
   // Example: BBR0, $50, +6
   // 0 8f 1 1 1 <opcode>
   // 1 50 1 0 1 <op1> i.e. ZP address
   // 2 01 1 0 1 <mem rd>
   // 3 01 1 0 1 <mem rd>
   // 4 06 1 0 1 <op2> i.e.
   // 5 20 1 0 1 (<branch taken penalty>)
   // 6          (<page crossed penalty>)
   //

   if (rockwell && (opcode & 0x0f) == 0x0f) {
      int operand = sample_q[2].data;
      // invert operand for BBR
      if (opcode <= 0x80) {
         operand ^= 0xff;
      }
      int bit = (opcode >> 4) & 7;
      // Figure out if the branch was taken
      if (operand & (1 << bit)) {
         // A taken bbr/bbs branch is 6 cycles, not 5
         cycle_count = 6;
         // A taken bbr/bbs branch that crosses a page boundary is 7 cycles
         if (PC >= 0) {
            int target =  (PC + 3) + ((int8_t)(op2));
            if ((target & 0xFF00) != ((PC + 3) & 0xff00)) {
               cycle_count = 7;
            }
         }
      }
   }

   // Account for extra cycles in a branch
   if (((opcode & 0x1f) == 0x10) || (opcode == 0x80)) {
      // Default to backards branches taken, forward not taken
      int taken = ((int8_t)op1) < 0;
      switch (opcode) {
      case 0x10: // BPL
         if (N >= 0) {
            taken = !N;
         }
         break;
      case 0x30: // BMI
         if (N >= 0) {
            taken = N;
         }
         break;
      case 0x50: // BVC
         if (V >= 0) {
            taken = !V;
         }
         break;
      case 0x70: // BVS
         if (V >= 0) {
            taken = V;
         }
         break;
      case 0x80: // BRA
         taken = 1;
         break;
      case 0x90: // BCC
         if (C >= 0) {
            taken = !C;
         }
         break;
      case 0xB0: // BCS
         if (C >= 0) {
            taken = C;
         }
         break;
      case 0xD0: // BNE
         if (Z >= 0) {
            taken = !Z;
         }
         break;
      case 0xF0: // BEQ
         if (Z >= 0) {
            taken = Z;
         }
         break;
      }
      if (taken) {
         // A taken branch is 3 cycles, not 2
         cycle_count = 3;
         // A taken branch that crosses a page boundary is 4 cycle
         if (PC >= 0) {
            int target =  (PC + 2) + ((int8_t)(op1));
            if ((target & 0xFF00) != ((PC + 2) & 0xff00)) {
               cycle_count = 4;
            }
         }
      }
   }

   // Master specific behaviour to remain in sync if rdy is not available
   if (master_nordy) {
      if (instr->len == 3) {
         if ((op2 == 0xfc) ||              // &FC00-&FCFF
             (op2 == 0xfd) ||              // &FD00-&FDFF
             (op2 == 0xfe && (
             ((op1 & 0xE0) == 0x00) ||     // &FE00-&FE1F
             ((op1 & 0xC0) == 0x40) ||     // &FE40-&FE7F
             ((op1 & 0xE0) == 0x80) ||     // &FE80-&FE9F
             ((op1 & 0xE0) == 0xC0)        // &FEC0-&FEDF
            ))) {
            // Use STA/STX/STA to determine 1MHz clock phase
            if (opcode == 0x8C || opcode == 0x8D || opcode == 0x8E) {
               if (sample_q[3].data == sample_q[4].data) {
                  int new_phase;
                  if (sample_q[3].data == sample_q[5].data) {
                     new_phase = 1;
                  } else {
                     new_phase = 0;
                  }
                  if (mhz1_phase != new_phase) {
                     //printf("correcting 1MHz phase\n");
                     mhz1_phase = new_phase;
                  }
               } else {
                  printf("fail: 1MHz access not extended as expected\n");
               }
            }
            // Correct cycle count based on expected cycle stretching behaviour
            if (opcode == 0x9D) {
               // STA abs, X which has an unfortunate dummy cycle
               cycle_count += 2 + mhz1_phase;
            } else {
               cycle_count += 1 + mhz1_phase;
            }
         }
      }
      // Toggle the phase every cycle
      mhz1_phase ^= (cycle_count & 1);
   }

   return cycle_count;
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

// ====================================================================
// Public Methods
// ====================================================================

static void em_6502_init(arguments_t *args) {

   switch (args->cpu_type) {
   case CPU_6502:
      instr_table = instr_table_6502;
      c02 = 0;
      rockwell = 0;
      break;
   case CPU_65C02_ROCKWELL:
      rockwell = 1;
      // fall through to
   case CPU_65C02:
      c02 = 1;
      instr_table = instr_table_65c02;
      break;
   default:
      printf("em_6502_init called with unsupported cpu_type (%d)\n", args->cpu_type);
      exit(1);
   }
   bbctube = args->bbctube;

   // Initialize Memory
   memory_init(0x10000);
   // Initialize the SP
   if (args->sp_reg >= 0) {
      S = args->sp_reg & 0xff;
   }

   // This flag tells the sync-less cycle count estimation to infer additional cycles on the master
   // It's needed when rdy is not being explicitely sampled
   master_nordy = (args->machine == MACHINE_MASTER) && (args->idx_rdy < 0);

   // If not supporting the Rockwell C02 extensions, tweak the cycle countes
   if (args->cpu_type == CPU_65C02) {
      // x7 (RMB/SMB): 5 cycles -> 1 cycles
      // xF (BBR/BBS): 5 cycles -> 1 cycles
      for (int i = 0x07; i <= 0xff; i+= 0x08) {
         instr_table[i].mnemonic = ILLEGAL;
         instr_table[i].mode     = IMP;
         instr_table[i].cycles   = 1;
         instr_table[i].optype   = READOP;
         instr_table[i].len      = 1;
      }
   }
   InstrType *instr = instr_table;
   for (int i = 0; i < 256; i++) {
      // Remove the undocumented instructions, if not supported
      if (instr->undocumented && !args->undocumented) {
         instr->mnemonic = ILLEGAL;
         instr->mode     = IMP;
         instr->cycles   = 1;
      }
      // Copy the length and format from the address mode, for efficiency
      instr->len = addr_mode_table[instr->mode].len;
      instr->fmt = addr_mode_table[instr->mode].fmt;
      instr++;
   }
}


static int em_6502_match_interrupt(sample_t *sample_q, int num_samples) {
   // Check we have enough valid samples
   if (num_samples < 7) {
      return 0;
   }
   // An interupt will write PCH, PCL, PSW in bus cycles 2,3,4
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

static int em_6502_count_cycles(sample_t *sample_q, int intr_seen) {
   if (sample_q[0].type == UNKNOWN) {
      return count_cycles_without_sync(sample_q, intr_seen);
   } else {
      return count_cycles_with_sync(sample_q);
   }
}

static void em_6502_reset(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   instruction->pc = -1;
   A = -1;
   X = -1;
   Y = -1;
   S = -1;
   N = -1;
   V = -1;
   D = -1;
   Z = -1;
   C = -1;
   I = 1;
   if (c02) {
      D = 0;
   }
   PC = (sample_q[num_cycles - 1].data << 8) + sample_q[num_cycles - 2].data;
}

static void em_6502_interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   interrupt(sample_q, num_cycles, instruction, 0);
}

static void em_6502_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction) {

   // Unpack the instruction bytes
   int opcode = sample_q[0].data;

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   int opcount = instr->len - 1;

   int op1 = (opcount < 1) ? 0 : sample_q[1].data;

   int op2 =
      (opcount < 2)             ? 0 :
      (opcode == 0x20)          ? sample_q[5].data :
      ((opcode & 0x0f) == 0x0f) ? sample_q[4].data : sample_q[2].data;

   // Memory Modelling: Instruction fetches
   if (PC >= 0) {
      int pc = PC;
      memory_read(opcode, pc++, MEM_INSTR);
      if (opcount >= 1) {
         memory_read(op1, pc++, MEM_INSTR);
      }
      if (opcount >= 2) {
         memory_read(op2, pc++, MEM_INSTR);
      }
   }

   // Save the instruction state
   instruction->opcode  = opcode;
   instruction->op1     = op1;
   instruction->op2     = op2;
   instruction->opcount = opcount;

   // Determine the current PC value
   if (opcode == 0x00) {
      // Now just pass BRK onto the interrupt handler
      interrupt(sample_q, num_cycles, instruction, 2);
      // And we are done
      interrupt(sample_q, num_cycles, instruction, 2);
      return;
   } else if (opcode == 0x20) {
      instruction->pc = (((sample_q[3].data << 8) + sample_q[4].data) - 2) & 0xffff;
   } else {
      instruction->pc = PC;
   }

   // Memory Modelling: Pointer indirection
   switch (instr->mode) {
   case IND:
   case INDY:
      // <opcode> <op1> <addrlo> <addrhi> [ <page crossing>] <operand>
      memory_read(sample_q[2].data,   op1             , MEM_POINTER);
      memory_read(sample_q[3].data, ((op1 + 1) & 0xff), MEM_POINTER);
      break;
   case INDX:
      // <opcode> <op1> <dummy> <addrlo> <addrhi> <operand>
      if (X >= 0) {
         memory_read(sample_q[3].data, ((op1 + X    ) & 0xff), MEM_POINTER);
         memory_read(sample_q[4].data, ((op1 + X + 1) & 0xff), MEM_POINTER);
      }
      break;
   case IND16:
      // e.g. JMP (1234)
      // <opcode=6C> <op1> <op2> <read new pcl> <read new pch>
      if (c02) {
         memory_read(sample_q[num_cycles - 2].data,  (op2 << 8) + op1              , MEM_POINTER);
         memory_read(sample_q[num_cycles - 1].data, ((op2 << 8) + op1 + 1) & 0xffff, MEM_POINTER);
      } else {
         memory_read(sample_q[num_cycles - 2].data, (op2 << 8) +   op1             , MEM_POINTER);
         memory_read(sample_q[num_cycles - 1].data, (op2 << 8) + ((op1 + 1) & 0xff), MEM_POINTER);
      }
      break;
   case IND1X:
      // JMP: <opcode=7C> <op1> <op2> <dummy> <read new pcl> <read new pch>
      if (X >= 0) {
         memory_read(sample_q[num_cycles - 2].data, ((op2 << 8) + op1 + X    ) & 0xffff, MEM_POINTER);
         memory_read(sample_q[num_cycles - 1].data, ((op2 << 8) + op1 + X + 1) & 0xffff, MEM_POINTER);
      }
      break;
   default:
      break;
   }

   if (instr->emulate) {

      int operand;
      if (instr->optype == RMWOP) {
         // e.g. <opcode> <op1> <op2> <read old> <write old> <write new>
         //      <opcode> <op1>       <read old> <write old> <write new>
         // Want to pick off the read
         operand = sample_q[num_cycles - 3].data;
      } else if (instr->optype == BRANCHOP) {
         // the operand is true if branch taken
         operand = (num_cycles != 2);
      } else if (opcode == 0x00) {
         // BRK: the operand is the data pushed to the stack (PCH, PCL, P)
         // <opcode> <op1> <write pch> <write pcl> <write p> <read rst> <read rsth>
         operand = (sample_q[2].data << 16) +  (sample_q[3].data << 8) + sample_q[4].data;
      } else if (opcode == 0x20) {
         // JSR: the operand is the data pushed to the stack (PCH, PCL)
         // <opcode> <op1> <read dummy> <write pch> <write pcl> <op2>
         operand = (sample_q[3].data << 8) + sample_q[4].data;
      } else if (opcode == 0x40) {
         // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
         // <opcode> <op1> <read dummy> <read p> <read pcl> <read pch>
         operand = (sample_q[5].data << 16) +  (sample_q[4].data << 8) + sample_q[3].data;
      } else if (opcode == 0x60) {
         // RTS: the operand is the data pulled from the stack (PCL, PCH)
         // <opcode> <op1> <read dummy> <read pcl> <read pch>
         operand = (sample_q[4].data << 8) + sample_q[3].data;
      } else if (instr->mode == IMM) {
         // Immediate addressing mode: the operand is the 2nd byte of the instruction
         operand = op1;
      } else if (instr->decimalcorrect && (D == 1)) {
         // read operations on the C02 that have an extra cycle added
         operand = sample_q[num_cycles - 2].data;
      } else {
         // default to using the last bus cycle as the operand
         operand = sample_q[num_cycles - 1].data;
      }

      // Operand 2 is the value written back in a store or read-modify-write
      // See RMW comment above for bus cycles
      uint32_t operand2 = operand;
      if (instr->optype == RMWOP || instr->optype == WRITEOP) {
         operand2 = sample_q[num_cycles - 1].data;
      }

      // For instructions that read or write memory, we need to work out the effective address
      // Note: not needed for stack operations, as S is used directly
      int ea = -1;
      int index;
      switch (instr->mode) {
      case ZP:
      case ZPR:
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
      default:
         // covers IMM, IMP, IMPA, IND16, IND1X, BRA
         break;
      }

      // Model memory reads
      if (ea >= 0 && (instr->optype == READOP || instr->optype == RMWOP)) {
         memory_read(operand, ea, MEM_DATA);
      }

      // Execute the instruction specific function
      // (This returns -1 if the result is unknown or invalid)
      int result = instr->emulate(operand, ea);

      if (instr->optype == WRITEOP || instr->optype == RMWOP) {

         // STA STX STY STZ
         // INC DEX ASL LSR ROL ROR
         // TSB TRB RMB SMB

         // Check result of instruction against bye
         if (result >= 0 && result != operand2) {
            failflag |= 1;
         }

         // Model memory writes based on result seen on bus
         if (ea >= 0) {
            memory_write(operand2,  ea, MEM_DATA);
         }
      }
   }

   // Look for control flow changes and update the PC
   if (opcode == 0x40 || opcode == 0x6c || opcode == 0x7c) {
      // RTI, JMP (ind), JMP (ind, X)
      PC = (sample_q[num_cycles - 1].data << 8) | sample_q[num_cycles - 2].data;
   } else if (opcode == 0x20 || opcode == 0x4c) {
      // JSR abs, JMP abs
      PC = op2 << 8 | op1;
   } else if (PC < 0) {
      // PC value is not known yet, everything below this point is relative
      PC = -1;
   } else if (opcode == 0x80) {
      // BRA
      PC = (PC + ((int8_t)(op1)) + 2) & 0xffff;
   } else if (rockwell && ((opcode & 0x0f) == 0x0f) && (num_cycles != 5)) {
      // BBR/BBS: op2 if taken
      PC = (PC + ((int8_t)(op2)) + 3) & 0xffff;
   } else if ((opcode & 0x1f) == 0x10 && num_cycles != 2) {
      // BXX: op1 if taken
      PC = (PC + ((int8_t)(op1)) + 2) & 0xffff;
   } else {
      // Otherwise, increment pc by length of instuction
      PC = (PC + opcount + 1) & 0xffff;
   }
}

static int em_6502_disassemble(char *buffer, instruction_t *instruction) {

   int numchars;
   int offset;
   char target[16];

   // Unpack the instruction bytes
   int opcode = instruction->opcode;
   int op1    = instruction->op1;
   int op2    = instruction->op2;
   int pc     = instruction->pc;

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
   case ZP:
   case ZPX:
   case ZPY:
   case INDX:
   case INDY:
   case IND:
      numchars = sprintf(buffer, fmt, mnemonic, op1);
      break;
   case ABS:
   case ABSX:
   case ABSY:
   case IND16:
   case IND1X:
      numchars = sprintf(buffer, fmt, mnemonic, op1, op2);
      break;
   default:
      numchars = 0;
   }

   return numchars;
}

static int em_6502_get_PC() {
   return PC;
}

static int em_6502_get_PB() {
   return 0;
}

static int em_6502_read_memory(int address) {
   return memory_read_raw(address);
}

static char *em_6502_get_state(char *buffer) {
   strcpy(buffer, default_state);
   if (A >= 0) {
      write_hex2(buffer + OFFSET_A, A);
   }
   if (X >= 0) {
      write_hex2(buffer + OFFSET_X, X);
   }
   if (Y >= 0) {
      write_hex2(buffer + OFFSET_Y, Y);
   }
   if (S >= 0) {
      write_hex2(buffer + OFFSET_S, S);
   }
   if (N >= 0) {
      buffer[OFFSET_N] = '0' + N;
   }
   if (V >= 0) {
      buffer[OFFSET_V] = '0' + V;
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
   return buffer + OFFSET_END;
}

static int em_6502_get_and_clear_fail() {
   int ret = failflag;
   failflag = 0;
   return ret;
}

cpu_emulator_t em_6502 = {
   .init = em_6502_init,
   .match_interrupt = em_6502_match_interrupt,
   .count_cycles = em_6502_count_cycles,
   .reset = em_6502_reset,
   .interrupt = em_6502_interrupt,
   .emulate = em_6502_emulate,
   .disassemble = em_6502_disassemble,
   .get_PC = em_6502_get_PC,
   .get_PB = em_6502_get_PB,
   .read_memory = em_6502_read_memory,
   .get_state = em_6502_get_state,
   .get_and_clear_fail = em_6502_get_and_clear_fail,
};

// ====================================================================
// Individual Instructions
// ====================================================================

static int op_ADC(operand_t operand, ea_t ea) {
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode ADC
         int al;
         int ah;
         uint8_t tmp;
         ah = 0;
         Z = N = 0;
         tmp = A + operand + (C ? 1 : 0);
         if (!tmp) {
            Z = 1;
         }
         al = (A & 0xF) + (operand & 0xF) + (C ? 1 : 0);
         if (al > 9) {
            al -= 10;
            al &= 0xF;
            ah = 1;
         }
         ah += ((A >> 4) + (operand >> 4));
         if (ah & 8) {
            N = 1;
         }
         V = (((ah << 4) ^ A) & 128) && !((A ^ operand) & 128);
         C = 0;
         if (ah > 9) {
            C = 1;
            ah -= 10;
            ah &= 0xF;
         }
         A = (al & 0xF) | (ah << 4);
         // On 65C02 ADC, only the NZ flags are different to the 6502
         if (c02) {
            set_NZ(A);
         }
      } else {
         // Normal mode ADC
         int tmp = A + operand + C;
         C = (tmp >> 8) & 1;
         V = (((A ^ operand) & 0x80) == 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
         set_NZ(A);
      }
   } else {
      A = -1;
      set_NVZC_unknown();
   }
   return -1;
}

static int op_AND(operand_t operand, ea_t ea) {
   if (A >= 0) {
      A = A & operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_ASLA(operand_t operand, ea_t ea) {
   if (A >= 0) {
      C = (A >> 7) & 1;
      A = (A << 1) & 255;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_ASL(operand_t operand, ea_t ea) {
   C = (operand >> 7) & 1;
   int tmp = (operand << 1) & 255;
   set_NZ(tmp);
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
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
   return -1;
}

static int op_BIT(operand_t operand, ea_t ea) {
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
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
   if (A >= 0) {
      int tmp = A - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_CPX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      int tmp = X - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_CPY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      int tmp = Y - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_DECA(operand_t operand, ea_t ea) {
   if (A >= 0) {
      A = (A - 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_DEC(operand_t operand, ea_t ea) {
   int tmp = (operand - 1) & 255;
   set_NZ(tmp);
   return tmp;
}

static int op_DEX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      X = (X - 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_DEY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      Y = (Y - 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_EOR(operand_t operand, ea_t ea) {
   if (A >= 0) {
      A = A ^ operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_INCA(operand_t operand, ea_t ea) {
   if (A >= 0) {
      A = (A + 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_INC(operand_t operand, ea_t ea) {
   int tmp = (operand + 1) & 255;
   set_NZ(tmp);
   return tmp;
}

static int op_INX(operand_t operand, ea_t ea) {
   if (X >= 0) {
      X = (X + 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_INY(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      Y = (Y + 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_JSR(operand_t operand, ea_t ea) {
   // JSR: the operand is the data pushed to the stack (PCH, PCL)
   push16(operand);
   // The +1 is handled elsewhere
   PC = operand & 0xffff;
   return -1;
}

static int op_LDA(operand_t operand, ea_t ea) {
   A = operand;
   set_NZ(A);
   return -1;
}

static int op_LDX(operand_t operand, ea_t ea) {
   X = operand;
   set_NZ(X);
   return -1;
}

static int op_LDY(operand_t operand, ea_t ea) {
   Y = operand;
   set_NZ(Y);
   return -1;
}

static int op_LSRA(operand_t operand, ea_t ea) {
   if (A >= 0) {
      C = A & 1;
      A = A >> 1;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

static int op_LSR(operand_t operand, ea_t ea) {
   C = operand & 1;
   int tmp = operand >> 1;
   set_NZ(tmp);
   return tmp;
}

static int op_ORA(operand_t operand, ea_t ea) {
   if (A >= 0) {
      A = A | operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
   return -1;
}

static int op_PHA(operand_t operand, ea_t ea) {
   push8(operand);
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
   push8(operand);
   op_STX(operand, -1);
   return -1;
}

static int op_PHY(operand_t operand, ea_t ea) {
   push8(operand);
   op_STY(operand, -1);
   return -1;
}

static int op_PLA(operand_t operand, ea_t ea) {
   A = operand;
   set_NZ(A);
   pop8(operand);
   return -1;
}

static int op_PLP(operand_t operand, ea_t ea) {
   set_FLAGS(operand);
   pop8(operand);
   return -1;
}

static int op_PLX(operand_t operand, ea_t ea) {
   X = operand;
   set_NZ(X);
   pop8(operand);
   return -1;
}

static int op_PLY(operand_t operand, ea_t ea) {
   Y = operand;
   set_NZ(Y);
   pop8(operand);
   return -1;
}

static int op_ROLA(operand_t operand, ea_t ea) {
   if (A >= 0 && C >= 0) {
      int tmp = (A << 1) + C;
      C = (tmp >> 8) & 1;
      A = tmp & 255;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
   return -1;
}

static int op_ROL(operand_t operand, ea_t ea) {
   if (C >= 0) {
      int tmp = (operand << 1) + C;
      C = (tmp >> 8) & 1;
      tmp = tmp & 255;
      set_NZ(tmp);
      return tmp;
   } else {
      set_NZC_unknown();
      return -1;
   }
}

static int op_RORA(operand_t operand, ea_t ea) {
   if (A >= 0 && C >= 0) {
      int tmp = (A >> 1) + (C << 7);
      C = A & 1;
      A = tmp;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
   return -1;
}

static int op_ROR(operand_t operand, ea_t ea) {
   if (C >= 0) {
      int tmp = (operand >> 1) + (C << 7);
      C = operand & 1;
      set_NZ(tmp);
      return tmp;
   } else {
      set_NZC_unknown();
      return -1;
   }
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
   return -1;
}

static int op_SBC(operand_t operand, ea_t ea) {
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode SBC
         if (c02) {
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
            set_NZ(A);
         } else {
            int al;
            int ah;
            int hc = 0;
            uint8_t tmp = A - operand - ((C) ? 0 : 1);
            Z = N = 0;
            if (!(tmp)) {
               Z = 1;
            }
            al = (A & 15) - (operand & 15) - ((C) ? 0 : 1);
            if (al & 16) {
               al -= 6;
               al &= 0xF;
               hc = 1;
            }
            ah = (A >> 4) - (operand >> 4);
            if (hc) {
               ah--;
            }
            if ((A - (operand + ((C) ? 0 : 1))) & 0x80) {
               N = 1;
            }
            V = ((A ^ operand) & 0x80) && ((A ^ tmp) & 0x80);
            C = 1;
            if (ah & 16) {
               C = 0;
               ah -= 6;
               ah &= 0xF;
            }
            A = (al & 0xF) | ((ah & 0xF) << 4);
         }
      } else {
         // Normal mode SBC
         int tmp = A - operand - (1 - C);
         C = 1 - ((tmp >> 8) & 1);
         V = (((A ^ operand) & 0x80) != 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
         set_NZ(A);
      }
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
   if (A >= 0) {
      if (operand != A) {
         failflag = 1;
      }
   }
   A = operand;
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
   return 0;
}

static int op_TAX(operand_t operand, ea_t ea) {
   if (A >= 0) {
      X = A;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_TAY(operand_t operand, ea_t ea) {
   if (A >= 0) {
      Y = A;
      set_NZ(Y);
   } else {
      Y = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_TSB(operand_t operand, ea_t ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      return operand | A;
   } else {
      Z = -1;
      return -1;
   }
}
static int op_TRB(operand_t operand, ea_t ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      return operand & ~A;
   } else {
      Z = -1;
      return -1;
   }
}

static int op_TSX(operand_t operand, ea_t ea) {
   if (S >= 0) {
      X = S;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_TXA(operand_t operand, ea_t ea) {
   if (X >= 0) {
      A = X;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_TXS(operand_t operand, ea_t ea) {
   if (X >= 0) {
      S = X;
   } else {
      S = -1;
   }
   return -1;
}

static int op_TYA(operand_t operand, ea_t ea) {
   if (Y >= 0) {
      A = Y;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
   return -1;
}

static int op_RMB(operand_t operand, ea_t ea) {
   return operand;
}

static int op_SMB(operand_t operand, ea_t ea) {
   return operand;
}

// ====================================================================
// Opcode Tables
// ====================================================================

static InstrType instr_table_65c02[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, 0, OTHER,    0},
   /* 01 */   { "ORA",  0, INDX  , 6, 0, READOP,   op_ORA},
   /* 02 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* 03 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 04 */   { "TSB",  0, ZP    , 5, 0, RMWOP,    op_TSB},
   /* 05 */   { "ORA",  0, ZP    , 3, 0, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, 0, RMWOP,    op_ASL},
   /* 07 */   { "RMB0", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 08 */   { "PHP",  0, IMP   , 3, 0, OTHER,    op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, 0, OTHER,    op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, 0, OTHER,    op_ASLA},
   /* 0B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 0C */   { "TSB",  0, ABS   , 6, 0, RMWOP,    op_TSB},
   /* 0D */   { "ORA",  0, ABS   , 4, 0, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, 0, RMWOP,    op_ASL},
   /* 0F */   { "BBR0", 0, ZPR   , 5, 0, READOP,   0},
   /* 10 */   { "BPL",  0, BRA   , 2, 0, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, 0, READOP,   op_ORA},
   /* 12 */   { "ORA",  0, IND   , 5, 0, READOP,   op_ORA},
   /* 13 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 14 */   { "TRB",  0, ZP    , 5, 0, RMWOP,    op_TRB},
   /* 15 */   { "ORA",  0, ZPX   , 4, 0, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, 0, RMWOP,    op_ASL},
   /* 17 */   { "RMB1", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 18 */   { "CLC",  0, IMP   , 2, 0, OTHER,    op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, 0, READOP,   op_ORA},
   /* 1A */   { "INC",  0, IMPA  , 2, 0, OTHER,    op_INCA},
   /* 1B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 1C */   { "TRB",  0, ABS   , 6, 0, RMWOP,    op_TRB},
   /* 1D */   { "ORA",  0, ABSX  , 4, 0, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 6, 0, RMWOP,    op_ASL},
   /* 1F */   { "BBR1", 0, ZPR   , 5, 0, READOP,   0},
   /* 20 */   { "JSR",  0, ABS   , 6, 0, OTHER,    op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, 0, READOP,   op_AND},
   /* 22 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* 23 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 24 */   { "BIT",  0, ZP    , 3, 0, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, 0, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, 0, RMWOP,    op_ROL},
   /* 27 */   { "RMB2", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 28 */   { "PLP",  0, IMP   , 4, 0, OTHER,    op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, 0, OTHER,    op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, 0, OTHER,    op_ROLA},
   /* 2B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 2C */   { "BIT",  0, ABS   , 4, 0, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, 0, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, 0, RMWOP,    op_ROL},
   /* 2F */   { "BBR2", 0, ZPR   , 5, 0, READOP,   0},
   /* 30 */   { "BMI",  0, BRA   , 2, 0, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, 0, READOP,   op_AND},
   /* 32 */   { "AND",  0, IND   , 5, 0, READOP,   op_AND},
   /* 33 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 34 */   { "BIT",  0, ZPX   , 4, 0, READOP,   op_BIT},
   /* 35 */   { "AND",  0, ZPX   , 4, 0, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, 0, RMWOP,    op_ROL},
   /* 37 */   { "RMB3", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 38 */   { "SEC",  0, IMP   , 2, 0, OTHER,    op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, 0, READOP,   op_AND},
   /* 3A */   { "DEC",  0, IMPA  , 2, 0, OTHER,    op_DECA},
   /* 3B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 3C */   { "BIT",  0, ABSX  , 4, 0, READOP,   op_BIT},
   /* 3D */   { "AND",  0, ABSX  , 4, 0, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 6, 0, RMWOP,    op_ROL},
   /* 3F */   { "BBR3", 0, ZPR   , 5, 0, READOP,   0},
   /* 40 */   { "RTI",  0, IMP   , 6, 0, OTHER,    op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, 0, READOP,   op_EOR},
   /* 42 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* 43 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 44 */   { "NOP",  0, ZP    , 3, 0, OTHER,    0},
   /* 45 */   { "EOR",  0, ZP    , 3, 0, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, 0, RMWOP,    op_LSR},
   /* 47 */   { "RMB4", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 48 */   { "PHA",  0, IMP   , 3, 0, OTHER,    op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, 0, OTHER,    op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, 0, OTHER,    op_LSRA},
   /* 4B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 4C */   { "JMP",  0, ABS   , 3, 0, OTHER,    0},
   /* 4D */   { "EOR",  0, ABS   , 4, 0, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, 0, RMWOP,    op_LSR},
   /* 4F */   { "BBR4", 0, ZPR   , 5, 0, READOP,   0},
   /* 50 */   { "BVC",  0, BRA   , 2, 0, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, 0, READOP,   op_EOR},
   /* 52 */   { "EOR",  0, IND   , 5, 0, READOP,   op_EOR},
   /* 53 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 54 */   { "NOP",  0, ZPX   , 4, 0, OTHER,    0},
   /* 55 */   { "EOR",  0, ZPX   , 4, 0, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, 0, RMWOP,    op_LSR},
   /* 57 */   { "RMB5", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 58 */   { "CLI",  0, IMP   , 2, 0, OTHER,    op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, 0, READOP,   op_EOR},
   /* 5A */   { "PHY",  0, IMP   , 3, 0, OTHER,    op_PHY},
   /* 5B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 5C */   { "NOP",  0, ABS   , 8, 0, OTHER,    0},
   /* 5D */   { "EOR",  0, ABSX  , 4, 0, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 6, 0, RMWOP,    op_LSR},
   /* 5F */   { "BBR5", 0, ZPR   , 5, 0, READOP,   0},
   /* 60 */   { "RTS",  0, IMP   , 6, 0, OTHER,    op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, 1, READOP,   op_ADC},
   /* 62 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* 63 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 64 */   { "STZ",  0, ZP    , 3, 0, WRITEOP,  op_STZ},
   /* 65 */   { "ADC",  0, ZP    , 3, 1, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, 0, RMWOP,    op_ROR},
   /* 67 */   { "RMB6", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 68 */   { "PLA",  0, IMP   , 4, 0, OTHER,    op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, 1, OTHER,    op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, 0, OTHER,    op_RORA},
   /* 6B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 6C */   { "JMP",  0, IND16 , 6, 0, OTHER,    0},
   /* 6D */   { "ADC",  0, ABS   , 4, 1, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, 0, RMWOP,    op_ROR},
   /* 6F */   { "BBR6", 0, ZPR   , 5, 0, READOP,   0},
   /* 70 */   { "BVS",  0, BRA   , 2, 0, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, 1, READOP,   op_ADC},
   /* 72 */   { "ADC",  0, IND   , 5, 1, READOP,   op_ADC},
   /* 73 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 74 */   { "STZ",  0, ZPX   , 4, 0, WRITEOP,  op_STZ},
   /* 75 */   { "ADC",  0, ZPX   , 4, 1, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, 0, RMWOP,    op_ROR},
   /* 77 */   { "RMB7", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 78 */   { "SEI",  0, IMP   , 2, 0, OTHER,    op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, 1, READOP,   op_ADC},
   /* 7A */   { "PLY",  0, IMP   , 4, 0, OTHER,    op_PLY},
   /* 7B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 7C */   { "JMP",  0, IND1X , 6, 0, OTHER,    0},
   /* 7D */   { "ADC",  0, ABSX  , 4, 1, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 6, 0, RMWOP,    op_ROR},
   /* 7F */   { "BBR7", 0, ZPR   , 5, 0, READOP,   0},
   /* 80 */   { "BRA",  0, BRA   , 3, 0, OTHER ,   0},
   /* 81 */   { "STA",  0, INDX  , 6, 0, WRITEOP,  op_STA},
   /* 82 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* 83 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 84 */   { "STY",  0, ZP    , 3, 0, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, 0, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, 0, WRITEOP,  op_STX},
   /* 87 */   { "SMB0", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* 88 */   { "DEY",  0, IMP   , 2, 0, OTHER,    op_DEY},
   /* 89 */   { "BIT",  0, IMM   , 2, 0, OTHER,    op_BIT_IMM},
   /* 8A */   { "TXA",  0, IMP   , 2, 0, OTHER,    op_TXA},
   /* 8B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 8C */   { "STY",  0, ABS   , 4, 0, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, 0, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, 0, WRITEOP,  op_STX},
   /* 8F */   { "BBS0", 0, ZPR   , 5, 0, READOP,   0},
   /* 90 */   { "BCC",  0, BRA   , 2, 0, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, 0, WRITEOP,  op_STA},
   /* 92 */   { "STA",  0, IND   , 5, 0, WRITEOP,  op_STA},
   /* 93 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 94 */   { "STY",  0, ZPX   , 4, 0, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, 0, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, 0, WRITEOP,  op_STX},
   /* 97 */   { "SMB1", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* 98 */   { "TYA",  0, IMP   , 2, 0, OTHER,    op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, 0, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, 0, OTHER,    op_TXS},
   /* 9B */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* 9C */   { "STZ",  0, ABS   , 4, 0, WRITEOP,  op_STZ},
   /* 9D */   { "STA",  0, ABSX  , 5, 0, WRITEOP,  op_STA},
   /* 9E */   { "STZ",  0, ABSX  , 5, 0, WRITEOP,  op_STZ},
   /* 9F */   { "BBS1", 0, ZPR   , 5, 0, READOP,   0},
   /* A0 */   { "LDY",  0, IMM   , 2, 0, OTHER,    op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, 0, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, 0, OTHER,    op_LDX},
   /* A3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* A4 */   { "LDY",  0, ZP    , 3, 0, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, 0, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, 0, READOP,   op_LDX},
   /* A7 */   { "SMB2", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* A8 */   { "TAY",  0, IMP   , 2, 0, OTHER,    op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, 0, OTHER,    op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, 0, OTHER,    op_TAX},
   /* AB */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* AC */   { "LDY",  0, ABS   , 4, 0, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, 0, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, 0, READOP,   op_LDX},
   /* AF */   { "BBS2", 0, ZPR   , 5, 0, READOP,   0},
   /* B0 */   { "BCS",  0, BRA   , 2, 0, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, 0, READOP,   op_LDA},
   /* B2 */   { "LDA",  0, IND   , 5, 0, READOP,   op_LDA},
   /* B3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* B4 */   { "LDY",  0, ZPX   , 4, 0, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, 0, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, 0, READOP,   op_LDX},
   /* B7 */   { "SMB3", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* B8 */   { "CLV",  0, IMP   , 2, 0, OTHER,    op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, 0, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, 0, OTHER,    op_TSX},
   /* BB */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* BC */   { "LDY",  0, ABSX  , 4, 0, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, 0, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, 0, READOP,   op_LDX},
   /* BF */   { "BBS3", 0, ZPR   , 5, 0, READOP,   0},
   /* C0 */   { "CPY",  0, IMM   , 2, 0, OTHER,    op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, 0, READOP,   op_CMP},
   /* C2 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* C3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* C4 */   { "CPY",  0, ZP    , 3, 0, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, 0, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, 0, RMWOP,    op_DEC},
   /* C7 */   { "SMB4", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* C8 */   { "INY",  0, IMP   , 2, 0, OTHER,    op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, 0, OTHER,    op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, 0, OTHER,    op_DEX},
   /* CB */   { "WAI",  0, IMP   , 1, 0, OTHER,    0},        // WD65C02=3
   /* CC */   { "CPY",  0, ABS   , 4, 0, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, 0, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, 0, RMWOP,    op_DEC},
   /* CF */   { "BBS4", 0, ZPR   , 5, 0, READOP,   0},
   /* D0 */   { "BNE",  0, BRA   , 2, 0, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, 0, READOP,   op_CMP},
   /* D2 */   { "CMP",  0, IND   , 5, 0, READOP,   op_CMP},
   /* D3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* D4 */   { "NOP",  0, ZPX   , 4, 0, OTHER,    0},
   /* D5 */   { "CMP",  0, ZPX   , 4, 0, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, 0, RMWOP,    op_DEC},
   /* D7 */   { "SMB5", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* D8 */   { "CLD",  0, IMP   , 2, 0, OTHER,    op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, 0, READOP,   op_CMP},
   /* DA */   { "PHX",  0, IMP   , 3, 0, OTHER,    op_PHX},
   /* DB */   { "STP",  0, IMP   , 1, 0, OTHER,    0},        // WD65C02=3
   /* DC */   { "NOP",  0, ABS   , 4, 0, OTHER,    0},
   /* DD */   { "CMP",  0, ABSX  , 4, 0, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, 0, RMWOP,    op_DEC},
   /* DF */   { "BBS5", 0, ZPR   , 5, 0, READOP,   0},
   /* E0 */   { "CPX",  0, IMM   , 2, 0, OTHER,    op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, 1, READOP,   op_SBC},
   /* E2 */   { "NOP",  0, IMM   , 2, 0, OTHER,    0},
   /* E3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* E4 */   { "CPX",  0, ZP    , 3, 0, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, 1, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, 0, RMWOP,    op_INC},
   /* E7 */   { "SMB6", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* E8 */   { "INX",  0, IMP   , 2, 0, OTHER,    op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, 1, OTHER,    op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, 0, OTHER,    0},
   /* EB */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* EC */   { "CPX",  0, ABS   , 4, 0, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, 1, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, 0, RMWOP,    op_INC},
   /* EF */   { "BBS6", 0, ZPR   , 5, 0, READOP,   0},
   /* F0 */   { "BEQ",  0, BRA   , 2, 0, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, 1, READOP,   op_SBC},
   /* F2 */   { "SBC",  0, IND   , 5, 1, READOP,   op_SBC},
   /* F3 */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* F4 */   { "NOP",  0, ZPX   , 4, 0, OTHER,    0},
   /* F5 */   { "SBC",  0, ZPX   , 4, 1, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, 0, RMWOP,    op_INC},
   /* F7 */   { "SMB7", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* F8 */   { "SED",  0, IMP   , 2, 0, OTHER,    op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, 1, READOP,   op_SBC},
   /* FA */   { "PLX",  0, IMP   , 4, 0, OTHER,    op_PLX},
   /* FB */   { "NOP",  0, IMP   , 1, 0, OTHER,    0},
   /* FC */   { "NOP",  0, ABS   , 4, 0, OTHER,    0},
   /* FD */   { "SBC",  0, ABSX  , 4, 1, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, 0, RMWOP,    op_INC},
   /* FF */   { "BBS7", 0, ZPR   , 5, 0, READOP,   0}
};

static InstrType instr_table_6502[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, 0, OTHER,    0},
   /* 01 */   { "ORA",  0, INDX  , 6, 0, READOP,   op_ORA},
   /* 02 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 03 */   { "SLO",  1, INDX  , 8, 0, READOP,   0},
   /* 04 */   { "NOP",  1, ZP    , 3, 0, OTHER,    0},
   /* 05 */   { "ORA",  0, ZP    , 3, 0, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, 0, RMWOP,    op_ASL},
   /* 07 */   { "SLO",  1, ZP    , 5, 0, READOP,   0},
   /* 08 */   { "PHP",  0, IMP   , 3, 0, OTHER,    op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, 0, OTHER,    op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, 0, OTHER,    op_ASLA},
   /* 0B */   { "ANC",  1, IMM   , 2, 0, OTHER,    0},
   /* 0C */   { "NOP",  1, ABS   , 4, 0, OTHER,    0},
   /* 0D */   { "ORA",  0, ABS   , 4, 0, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, 0, RMWOP,    op_ASL},
   /* 0F */   { "SLO",  1, ABS   , 6, 0, READOP,   0},
   /* 10 */   { "BPL",  0, BRA   , 2, 0, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, 0, READOP,   op_ORA},
   /* 12 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 13 */   { "SLO",  1, INDY  , 8, 0, READOP,   0},
   /* 14 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* 15 */   { "ORA",  0, ZPX   , 4, 0, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, 0, RMWOP,    op_ASL},
   /* 17 */   { "SLO",  1, ZPX   , 6, 0, READOP,   0},
   /* 18 */   { "CLC",  0, IMP   , 2, 0, OTHER,    op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, 0, READOP,   op_ORA},
   /* 1A */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* 1B */   { "SLO",  1, ABSY  , 7, 0, READOP,   0},
   /* 1C */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* 1D */   { "ORA",  0, ABSX  , 4, 0, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 7, 0, RMWOP,    op_ASL},
   /* 1F */   { "SLO",  1, ABSX  , 4, 0, READOP,   0},
   /* 20 */   { "JSR",  0, ABS   , 6, 0, OTHER,    op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, 0, READOP,   op_AND},
   /* 22 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 23 */   { "RLA",  1, INDX  , 8, 0, READOP,   0},
   /* 24 */   { "BIT",  0, ZP    , 3, 0, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, 0, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, 0, RMWOP,    op_ROL},
   /* 27 */   { "RLA",  1, ZP    , 5, 0, READOP,   0},
   /* 28 */   { "PLP",  0, IMP   , 4, 0, OTHER,    op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, 0, OTHER,    op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, 0, OTHER,    op_ROLA},
   /* 2B */   { "ANC",  1, IMM   , 2, 0, OTHER,    0},
   /* 2C */   { "BIT",  0, ABS   , 4, 0, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, 0, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, 0, RMWOP,    op_ROL},
   /* 2F */   { "RLA",  1, ABS   , 6, 0, READOP,   0},
   /* 30 */   { "BMI",  0, BRA   , 2, 0, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, 0, READOP,   op_AND},
   /* 32 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 33 */   { "RLA",  1, INDY  , 8, 0, READOP,   0},
   /* 34 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* 35 */   { "AND",  0, ZPX   , 4, 0, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, 0, RMWOP,    op_ROL},
   /* 37 */   { "RLA",  1, ZPX   , 6, 0, READOP,   0},
   /* 38 */   { "SEC",  0, IMP   , 2, 0, OTHER,    op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, 0, READOP,   op_AND},
   /* 3A */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* 3B */   { "RLA",  1, ABSY  , 7, 0, READOP,   0},
   /* 3C */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* 3D */   { "AND",  0, ABSX  , 4, 0, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 7, 0, RMWOP,    op_ROL},
   /* 3F */   { "RLA",  1, ABSX  , 7, 0, READOP,   0},
   /* 40 */   { "RTI",  0, IMP   , 6, 0, OTHER,    op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, 0, READOP,   op_EOR},
   /* 42 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 43 */   { "SRE",  1, INDX  , 8, 0, READOP,   0},
   /* 44 */   { "NOP",  1, ZP    , 3, 0, OTHER,    0},
   /* 45 */   { "EOR",  0, ZP    , 3, 0, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, 0, RMWOP,    op_LSR},
   /* 47 */   { "SRE",  1, ZP    , 5, 0, READOP,   0},
   /* 48 */   { "PHA",  0, IMP   , 3, 0, OTHER,    op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, 0, OTHER,    op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, 0, OTHER,    op_LSRA},
   /* 4B */   { "ALR",  1, IMM   , 2, 0, OTHER,    0},
   /* 4C */   { "JMP",  0, ABS   , 3, 0, OTHER,    0},
   /* 4D */   { "EOR",  0, ABS   , 4, 0, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, 0, RMWOP,    op_LSR},
   /* 4F */   { "SRE",  1, ABS   , 6, 0, READOP,   0},
   /* 50 */   { "BVC",  0, BRA   , 2, 0, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, 0, READOP,   op_EOR},
   /* 52 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 53 */   { "SRE",  1, INDY  , 8, 0, READOP,   0},
   /* 54 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* 55 */   { "EOR",  0, ZPX   , 4, 0, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, 0, RMWOP,    op_LSR},
   /* 57 */   { "SRE",  1, ZPX   , 6, 0, READOP,   0},
   /* 58 */   { "CLI",  0, IMP   , 2, 0, OTHER,    op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, 0, READOP,   op_EOR},
   /* 5A */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* 5B */   { "SRE",  1, ABSY  , 7, 0, READOP,   0},
   /* 5C */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* 5D */   { "EOR",  0, ABSX  , 4, 0, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 7, 0, RMWOP,    op_LSR},
   /* 5F */   { "SRX",  1, ABSX  , 7, 0, READOP,   0},
   /* 60 */   { "RTS",  0, IMP   , 6, 0, OTHER,    op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, 0, READOP,   op_ADC},
   /* 62 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 63 */   { "RRA",  1, INDX  , 8, 0, READOP,   0},
   /* 64 */   { "NOP",  1, ZP    , 3, 0, OTHER,    0},
   /* 65 */   { "ADC",  0, ZP    , 3, 0, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, 0, RMWOP,    op_ROR},
   /* 67 */   { "RRA",  1, ZP    , 5, 0, READOP,   0},
   /* 68 */   { "PLA",  0, IMP   , 4, 0, OTHER,    op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, 0, OTHER,    op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, 0, OTHER,    op_RORA},
   /* 6B */   { "ARR",  1, IMM   , 2, 0, OTHER,    0},
   /* 6C */   { "JMP",  0, IND16 , 5, 0, OTHER,    0},
   /* 6D */   { "ADC",  0, ABS   , 4, 0, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, 0, RMWOP,    op_ROR},
   /* 6F */   { "RRA",  1, ABS   , 6, 0, READOP,   0},
   /* 70 */   { "BVS",  0, BRA   , 2, 0, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, 0, READOP,   op_ADC},
   /* 72 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 73 */   { "RRA",  1, INDY  , 8, 0, READOP,   0},
   /* 74 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* 75 */   { "ADC",  0, ZPX   , 4, 0, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, 0, RMWOP,    op_ROR},
   /* 77 */   { "RRA",  1, ZPX   , 6, 0, READOP,   0},
   /* 78 */   { "SEI",  0, IMP   , 2, 0, OTHER,    op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, 0, READOP,   op_ADC},
   /* 7A */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* 7B */   { "RRA",  1, ABSY  , 8, 0, READOP,   0},
   /* 7C */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* 7D */   { "ADC",  0, ABSX  , 4, 0, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 7, 0, RMWOP,    op_ROR},
   /* 7F */   { "RRA",  1, ABSX  , 8, 0, READOP,   0},
   /* 80 */   { "NOP",  1, IMM   , 2, 0, OTHER,    0},
   /* 81 */   { "STA",  0, INDX  , 6, 0, WRITEOP,  op_STA},
   /* 82 */   { "NOP",  1, IMM   , 2, 0, OTHER,    0},
   /* 83 */   { "SAX",  1, INDX  , 6, 0, READOP,   0},
   /* 84 */   { "STY",  0, ZP    , 3, 0, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, 0, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, 0, WRITEOP,  op_STX},
   /* 87 */   { "SAX",  1, ZP    , 3, 0, READOP,   0},
   /* 88 */   { "DEY",  0, IMP   , 2, 0, OTHER,    op_DEY},
   /* 89 */   { "NOP",  1, IMM   , 2, 0, OTHER,    0},
   /* 8A */   { "TXA",  0, IMP   , 2, 0, OTHER,    op_TXA},
   /* 8B */   { "XXA",  1, IMM   , 2, 0, OTHER,    0},
   /* 8C */   { "STY",  0, ABS   , 4, 0, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, 0, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, 0, WRITEOP,  op_STX},
   /* 8F */   { "SAX",  1, ABS   , 4, 0, READOP,   0},
   /* 90 */   { "BCC",  0, BRA   , 2, 0, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, 0, WRITEOP,  op_STA},
   /* 92 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* 93 */   { "AHX",  1, INDY  , 6, 0, READOP,   0},
   /* 94 */   { "STY",  0, ZPX   , 4, 0, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, 0, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, 0, WRITEOP,  op_STX},
   /* 97 */   { "SAX",  1, ZPY   , 4, 0, READOP,   0},
   /* 98 */   { "TYA",  0, IMP   , 2, 0, OTHER,    op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, 0, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, 0, OTHER,    op_TXS},
   /* 9B */   { "TAS",  1, ABS   , 5, 0, READOP,   0},
   /* 9C */   { "SHY",  1, ABSX  , 5, 0, READOP,   0},
   /* 9D */   { "STA",  0, ABSX  , 5, 0, WRITEOP,  op_STA},
   /* 9E */   { "SHX",  1, ABSY  , 5, 0, READOP,   0},
   /* 9F */   { "AHX",  1, ABSY  , 5, 0, READOP,   0},
   /* A0 */   { "LDY",  0, IMM   , 2, 0, OTHER,    op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, 0, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, 0, OTHER,    op_LDX},
   /* A3 */   { "LAX",  1, INDX  , 6, 0, READOP,   0},
   /* A4 */   { "LDY",  0, ZP    , 3, 0, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, 0, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, 0, READOP,   op_LDX},
   /* A7 */   { "LAX",  1, ZP    , 3, 0, READOP,   0},
   /* A8 */   { "TAY",  0, IMP   , 2, 0, OTHER,    op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, 0, OTHER,    op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, 0, OTHER,    op_TAX},
   /* AB */   { "LAX",  1, IMM   , 2, 0, OTHER,    0},
   /* AC */   { "LDY",  0, ABS   , 4, 0, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, 0, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, 0, READOP,   op_LDX},
   /* AF */   { "LAX",  1, ABS   , 4, 0, READOP,   0},
   /* B0 */   { "BCS",  0, BRA   , 2, 0, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, 0, READOP,   op_LDA},
   /* B2 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* B3 */   { "LAX",  1, INDY  , 5, 0, READOP,   0},
   /* B4 */   { "LDY",  0, ZPX   , 4, 0, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, 0, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, 0, READOP,   op_LDX},
   /* B7 */   { "LAX",  1, ZPY   , 4, 0, READOP,   0},
   /* B8 */   { "CLV",  0, IMP   , 2, 0, OTHER,    op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, 0, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, 0, OTHER,    op_TSX},
   /* BB */   { "LAS",  1, ABSY  , 4, 0, READOP,   0},
   /* BC */   { "LDY",  0, ABSX  , 4, 0, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, 0, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, 0, READOP,   op_LDX},
   /* BF */   { "LAX",  1, ABSY  , 4, 0, READOP,   0},
   /* C0 */   { "CPY",  0, IMM   , 2, 0, OTHER,    op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, 0, READOP,   op_CMP},
   /* C2 */   { "NOP",  1, IMP   , 0, 0, OTHER,    0},
   /* C3 */   { "DCP",  1, INDX  , 8, 0, READOP,   0},
   /* C4 */   { "CPY",  0, ZP    , 3, 0, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, 0, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, 0, RMWOP,    op_DEC},
   /* C7 */   { "DCP",  1, ZP    , 5, 0, READOP,   0},
   /* C8 */   { "INY",  0, IMP   , 2, 0, OTHER,    op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, 0, OTHER,    op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, 0, OTHER,    op_DEX},
   /* CB */   { "AXS",  1, IMM   , 2, 0, OTHER,    0},
   /* CC */   { "CPY",  0, ABS   , 4, 0, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, 0, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, 0, RMWOP,    op_DEC},
   /* CF */   { "DCP",  1, ABS   , 6, 0, READOP,   0},
   /* D0 */   { "BNE",  0, BRA   , 2, 0, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, 0, READOP,   op_CMP},
   /* D2 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* D3 */   { "DCP",  1, INDY  , 8, 0, READOP,   0},
   /* D4 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* D5 */   { "CMP",  0, ZPX   , 4, 0, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, 0, RMWOP,    op_DEC},
   /* D7 */   { "DCP",  1, ZPX   , 6, 0, READOP,   0},
   /* D8 */   { "CLD",  0, IMP   , 2, 0, OTHER,    op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, 0, READOP,   op_CMP},
   /* DA */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* DB */   { "DCP",  1, ABSY  , 7, 0, READOP,   0},
   /* DC */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* DD */   { "CMP",  0, ABSX  , 4, 0, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, 0, RMWOP,    op_DEC},
   /* DF */   { "DCP",  1, ABSX  , 7, 0, READOP,   0},
   /* E0 */   { "CPX",  0, IMM   , 2, 0, OTHER,    op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, 0, READOP,   op_SBC},
   /* E2 */   { "NOP",  1, IMP   , 0, 0, OTHER,    0},
   /* E3 */   { "ISC",  1, INDX  , 8, 0, READOP,   0},
   /* E4 */   { "CPX",  0, ZP    , 3, 0, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, 0, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, 0, RMWOP,    op_INC},
   /* E7 */   { "ISC",  1, ZP    , 5, 0, READOP,   0},
   /* E8 */   { "INX",  0, IMP   , 2, 0, OTHER,    op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, 0, OTHER,    op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, 0, OTHER,    0},
   /* EB */   { "SBC",  1, IMM   , 2, 0, OTHER,    0},
   /* EC */   { "CPX",  0, ABS   , 4, 0, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, 0, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, 0, RMWOP,    op_INC},
   /* EF */   { "ISC",  1, ABS   , 6, 0, READOP,   0},
   /* F0 */   { "BEQ",  0, BRA   , 2, 0, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, 0, READOP,   op_SBC},
   /* F2 */   { "KIL",  1, IMP   , 0, 0, OTHER,    0},
   /* F3 */   { "ISC",  1, INDY  , 8, 0, READOP,   0},
   /* F4 */   { "NOP",  1, ZPX   , 4, 0, OTHER,    0},
   /* F5 */   { "SBC",  0, ZPX   , 4, 0, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, 0, RMWOP,    op_INC},
   /* F7 */   { "ISC",  1, ZPX   , 6, 0, READOP,   0},
   /* F8 */   { "SED",  0, IMP   , 2, 0, OTHER,    op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, 0, READOP,   op_SBC},
   /* FA */   { "NOP",  1, IMP   , 2, 0, OTHER,    0},
   /* FB */   { "ISC",  1, ABSY  , 7, 0, READOP,   0},
   /* FC */   { "NOP",  1, ABSX  , 4, 0, OTHER,    0},
   /* FD */   { "SBC",  0, ABSX  , 4, 0, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, 0, RMWOP,    op_INC},
   /* FF */   { "ISC",  1, ABSX  , 7, 0, READOP,   0}
};
