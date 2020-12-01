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
   BRANCHOP
} OpType;

typedef struct {
   int len;
   const char *fmt;
} AddrModeType;

typedef struct {
   const char *mnemonic;
   int undocumented;
   AddrMode mode;
   int cycles;
   int decimalcorrect;
   OpType optype;
   void (*emulate)(int, int);
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
#define OFFSET_S   24
#define OFFSET_N   31
#define OFFSET_V   35
#define OFFSET_MS  39
#define OFFSET_XS  43
#define OFFSET_D   47
#define OFFSET_I   51
#define OFFSET_Z   55
#define OFFSET_C   59
#define OFFSET_E   63
#define OFFSET_END 64

static const char default_state[] = "A=???? X=???? Y=???? SP=???? N=? V=? M=? X=? D=? I=? Z=? C=? E=?";

static int c02;
static int bbctube;
static int master_nordy;

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
   {2,    "%1$s %2$s"},                // BRL
   {3,    "%1$s %2$02X,%3$02X"}        // BM
};

static const char *fmt_imm16 = "%1$s #%3$02X%2$02X";

// 6502 registers: -1 means unknown
static int A = -1;
static int X = -1;
static int Y = -1;
static int S = -1;
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
// TODO: these should start as unknown
static int MS = 1; // Accumulator and Memeory Size Flag
static int XS = 1; // Index Register Size Flag
static int E =  1; // Emulation Mode Flag, updated by XCE

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

static void op_STA(int operand, int ea);
static void op_STX(int operand, int ea);
static void op_STY(int operand, int ea);

// ====================================================================
// Helper Methods
// ====================================================================

static void memory_read(int data, int ea) {
   // TODO: allow memory bounds to be passed in as a command line parameter
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
      if (data < 0 || data > 255) {
         printf("memory modelling failed at %04x: illegal write of %02x\n", ea, data);
         failflag |= 1;
      } else {
         // printf("memory write: %04x = %02x\n", ea, data);
         memory[ea] = data;
      }
   }
   if (bbctube && ea >= 0xfee0 && ea <= 0xfee7) {
      tube_write(ea & 7, data);
   }
}

static int compare_NVDIZC(int operand) {
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

static void check_NVDIZC(int operand) {
   failflag |= compare_NVDIZC(operand);
}

static void set_NVDIZC(int operand) {
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


static void interrupt(int pc, int flags, int vector) {
   if (S >= 0) {
      // Push PCH
      memory_write((pc >> 8) & 0xff, 0x100 + S);
      S = (S - 1) & 255;
      // Push PCL
      memory_write(pc & 0xff, 0x100 + S);
      S = (S - 1) & 255;
      // Push P
      memory_write(flags, 0x100 + S);
      S = (S - 1) & 255;
   }
   check_NVDIZC(flags);
   set_NVDIZC(flags);
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

// ====================================================================
// Public Methods
// ====================================================================

static void em_65816_init(cpu_t cpu_type, int undocumented, int decode_bbctube, int mast_nordy) {
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
   master_nordy = mast_nordy;
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
            if (!compare_NVDIZC(sample_q[4].data)) {
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
   S = -1;
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
   MS = 1;
   XS = 1;
   // Program Counter
   PC = (sample_q[num_cycles - 1].data << 8) + sample_q[num_cycles - 2].data;
}

static void em_65816_interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   int pc   = (sample_q[2].data << 8) + sample_q[3].data;
   int flags = sample_q[4].data;
   int vector = (sample_q[6].data << 8) + sample_q[5].data;
   instruction->pc = pc;
   interrupt(pc, flags, vector);
}

static void em_65816_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction) {

   // Unpack the instruction bytes
   int opcode = sample_q[0].data;

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   int opcount = 0;
   // Immediate operands can be 16-bit
   // TODO: could also figure this out from the instruction length (num_cycles)
   if (instr->mode == IMM) {
      if (MS == 0 && instr->m_extra) {
         opcount = 1;
      }
      if (XS == 0 && instr->x_extra) {
         opcount = 1;
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
   if (opcode == 0x00) {
      if (E == 0) {
         // BRK: E=0 <opcode> <op1> <write pbr> <write pch> <write pcl> <write p> <read rst> <read rsth>
         instruction->pc = (((sample_q[3].data << 8) + sample_q[4].data) - 2) & 0xffff;
         instruction->pb = sample_q[2].data;
      } else {
         // BRK: E=1 <opcode> <op1> <write pch> <write pcl> <write p> <read rst> <read rsth>
         instruction->pc = (((sample_q[3].data << 8) + sample_q[2].data) - 2) & 0xffff;
         instruction->pb = PB;
      }
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

   if (instr->emulate) {

      int operand;
      if (instr->optype == RMWOP) {
         // e.g. <opcode> <op1> <op2> <read> <write> <write>
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
         // <opcode> <op1> <op2> <read dummy> <write pch> <write pcl>
         operand = (sample_q[4].data << 8) + sample_q[5].data;
      } else if (opcode == 0x22) {
         // JSL: the operand is the data pushed to the stack (PCB, PCH, PCL)
         // <opcode> <op1> <op2> <write pbr> <read dummy> <op3> <write pch> <write pcl>
         operand = (sample_q[3].data << 16) + (sample_q[6].data << 8) + sample_q[7].data;
      } else if (opcode == 0x40) {
         // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
         // <opcode> <op1> <read dummy> <read p> <read pcl> <read pch>
         operand = (sample_q[3].data << 16) +  (sample_q[4].data << 8) + sample_q[5].data;
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
      } else if (instr->decimalcorrect && (D == 1)) {
         // read operations on the C02 that have an extra cycle added
         operand = sample_q[num_cycles - 2].data;
      } else if (instr->optype == TSBTRBOP) {
         // For TSB/TRB, <opcode> <op1> <read> <dummy> <write> the operand is the <read>
         operand = sample_q[num_cycles - 3].data;
      } else {
         // default to using the last bus cycle as the operand
         operand = sample_q[num_cycles - 1].data;
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
      default:
         break;
      }

      if (opcode == 0x00) {
         ea = (sample_q[6].data << 8) + sample_q[5].data;
      }

      instr->emulate(operand, ea);
   }

   // TODO: Push the PC updates into the emulation code

   // Look for control flow changes and update the PC
   if (opcode == 0x40 || opcode == 0x00 || opcode == 0x6c || opcode == 0x7c) {
      // RTI, BRK, INTR, JMP (ind), JMP (ind, X), IRQ/NMI/RST
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
   } else if (opcode == 0x80) {
      // BRA
      PC += ((int8_t)(op1)) + 2;
      PC &= 0xffff;
   } else if (opcode == 0x82) {
      // BRL
      PC += ((int16_t)((op2 << 8) + op1)) + 3;
      PC &= 0xffff;
   } else if ((opcode & 0x1f) == 0x10 && num_cycles != 2) {
      // BXX: op1 if taken
      PC += ((int8_t)(op1)) + 2;
      PC &= 0xffff;
   } else {
      // Otherwise, increment pc by length of instuction
      PC += opcount + 1;
      PC &= 0xffff;
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
   return (PB << 24) + PC;
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
   if (S >= 0) {
      write_hex4(buffer + OFFSET_S, S);
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
   .read_memory = em_65816_read_memory,
   .get_state = em_65816_get_state,
   .get_and_clear_fail = em_65816_get_and_clear_fail,
};

// ====================================================================
// 65816 specific instructions
// ====================================================================

// Coprocessor

static void op_COP(int operand, int ea) {
   D = 0;
   I = 0;
}

// Push Effective Address
static void op_PEA(int operand, int ea) {
   if (S >= 0) {
      memory_write((operand >> 8) & 255, 0x100 + S);
      S = (S - 1) & 255;
      memory_write(operand & 255, 0x100 + S);
      S = (S - 1) & 255;
   }
}

static void op_PEI(int operand, int ea) {
   // TODO
}

static void op_PER(int operand, int ea) {
   // TODO
}

// Push Data Bank Register
static void op_PHB(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   if (DB >= 0) {
      if (operand != DB) {
         failflag = 1;
      }
   }
   DB = operand;
}

// Push Program Bank Register
static void op_PHK(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   if (PB >= 0) {
      if (operand != PB) {
         failflag = 1;
      }
   }
   PB = operand;
}

// Push Direct Page Register
static void op_PHD(int operand, int ea) {
   if (S >= 0) {
      memory_write((operand >> 8) & 255, 0x100 + S);
      S = (S - 1) & 255;
      memory_write(operand & 255, 0x100 + S);
      S = (S - 1) & 255;
   }
   if (DP >= 0) {
      if (operand != DP) {
         failflag = 1;
      }
   }
   DP = operand;
}

// Pull Data Bank Register
static void op_PLB(int operand, int ea) {
   DB = operand;
   set_NZ(DB);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

// Pull Direct Page Register
static void op_PLD(int operand, int ea) {
   DP = operand;
   set_NZ(DP);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);
   }
}


// Block Move (Decrementing)
static void op_MVP(int operand, int ea) {
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
}

// Block Move (Incrementing)
static void op_MVN(int operand, int ea) {
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
}

// Transfer Transfer C accumulator to Direct Page register
static void op_TCD(int operand, int ea) {
   if (B >= 0 && A >= 0) {
      DP = (B << 8) + A;
      // TODO: Need to pick the right sign bit
      set_NZ(DP);
   } else {
      DP = -1;
      set_NZ_unknown();
   }
}

// Transfer Transfer C accumulator to Stack pointer
static void op_TCS(int operand, int ea) {
   if (B >= 0 && A >= 0) {
      S = (B << 8) + A;
   } else {
      S = -1;
   }
}

// Transfer Transfer Direct Page register to C accumulator
static void op_TDC(int operand, int ea) {
   if (DP >= 0) {
      A = DP & 255;
      B = (DP >> 8) & 255;
      // TODO: Need to pick the right sign bit
      set_NZ(DP);
   } else {
      A = -1;
      B = -1;
      set_NZ_unknown();
   }
}

// Transfer Transfer Stack pointer to C accumulator
static void op_TSC(int operand, int ea) {
   if (S >= 0) {
      A = S & 255;
      B = (S >> 8) & 255;
      // TODO: Need to pick the right sign bit
      set_NZ(S);
   } else {
      A = -1;
      B = -1;
      set_NZ_unknown();
   }
}

static void op_TXY(int operand, int ea) {
   if (X >= 0) {
      Y = X;
      set_NZ(Y);
   } else {
      Y = -1;
      set_NZ_unknown();
   }
}

static void op_TYX(int operand, int ea) {
   if (Y >= 0) {
      X = Y;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
}

// Exchange A and B
static void op_XBA(int operand, int ea) {
   int tmp = A;
   B = A;
   A = tmp;
   if (A >= 0) {
      // Always based on the 8-bit result of A
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_XCE(int operand, int ea) {
   int tmp = C;
   C = E;
   E = tmp;
   if (E < 0) {
      MS = -1;
      XS = -1;
   } else if (E > 0) {
      MS = 1;
      XS = 1;
      X &= 0x00ff;
      Y &= 0x00ff;
      S &= 0x00ff;
      S |= 0x0100;
   }
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
static void op_REP(int operand, int ea) {
   repsep(operand, 0);
}

static void op_SEP(int operand, int ea) {
   repsep(operand, 1);
}

// Jump to Subroutine Long
static void op_JSL(int operand, int ea) {
   // JSR: the operand is the data pushed to the stack (PB, PCH, PCL)
   if (S >= 0) {
      memory_write((operand >> 16) & 255, 0x100 + S); // PB
      S = (S - 1) & 255;
      memory_write((operand >> 8) & 255, 0x100 + S);  // PCH
      S = (S - 1) & 255;
      memory_write(operand & 255, 0x100 + S);         // PCL
      S = (S - 1) & 255;
   }
}

// Return from Subroutine Long
static void op_RTL(int operand, int ea) {
   // RTL: the operand is the data pulled from the stack (PCL, PCH, PB)
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);           // PCL
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);    // PCH
      S = (S + 1) & 255;
      memory_read((operand >> 16) & 255, 0x100 + S);   // PB
   }
   // The +1 is handled elsewhere
   PC = operand & 0xffff;
   PB = (operand >> 16) & 0xff;
}

// ====================================================================
// 65816/6502 instructions
// ====================================================================

static void op_ADC(int operand, int ea) {
   memory_read(operand, ea);
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
}

static void op_AND(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A & operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_ASLA(int operand, int ea) {
   if (A >= 0) {
      C = (A >> 7) & 1;
      A = (A << 1) & 255;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_ASL(int operand, int ea) {
   memory_read(operand, ea);
   C = (operand >> 7) & 1;
   int tmp = (operand << 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_BCC(int branch_taken, int ea) {
   if (C >= 0) {
      if (C == branch_taken) {
         failflag = 1;
      }
   } else {
      C = 1 - branch_taken;
   }
}

static void op_BCS(int branch_taken, int ea) {
   if (C >= 0) {
      if (C != branch_taken) {
         failflag = 1;
      }
   } else {
      C = branch_taken;
   }
}

static void op_BNE(int branch_taken, int ea) {
   if (Z >= 0) {
      if (Z == branch_taken) {
         failflag = 1;
      }
   } else {
      Z = 1 - branch_taken;
   }
}

static void op_BEQ(int branch_taken, int ea) {
   if (Z >= 0) {
      if (Z != branch_taken) {
         failflag = 1;
        }
   } else {
      Z = branch_taken;
   }
}

static void op_BPL(int branch_taken, int ea) {
   if (N >= 0) {
      if (N == branch_taken) {
         failflag = 1;
      }
   } else {
      N = 1 - branch_taken;
   }
}

static void op_BMI(int branch_taken, int ea) {
   if (N >= 0) {
      if (N != branch_taken) {
         failflag = 1;
      }
   } else {
      N = branch_taken;
   }
}

static void op_BVC(int branch_taken, int ea) {
   if (V >= 0) {
      if (V == branch_taken) {
         failflag = 1;
      }
   } else {
      V = 1 - branch_taken;
   }
}

static void op_BVS(int branch_taken, int ea) {
   if (V >= 0) {
      if (V != branch_taken) {
         failflag = 1;
        }
   } else {
      V = branch_taken;
   }
}

static void op_BRK(int operand, int ea) {
   // BRK: the operand is the data pushed to the stack (PCH, PCL, P)
   int flags = operand & 0xff;
   int pc = (operand >> 8) & 0xffff;
   int vector = ea;
   interrupt(flags, pc, vector);
}

static void op_BIT_IMM(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_BIT(int operand, int ea) {
   memory_read(operand, ea);
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_CLC(int operand, int ea) {
   C = 0;
}

static void op_CLD(int operand, int ea) {
   D = 0;
}

static void op_CLI(int operand, int ea) {
   I = 0;
}

static void op_CLV(int operand, int ea) {
   V = 0;
}

static void op_CMP(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      int tmp = A - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_CPX(int operand, int ea) {
   memory_read(operand, ea);
   if (X >= 0) {
      int tmp = X - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_CPY(int operand, int ea) {
   memory_read(operand, ea);
   if (Y >= 0) {
      int tmp = Y - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_DECA(int operand, int ea) {
   if (A >= 0) {
      A = (A - 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEC(int operand, int ea) {
   memory_read(operand, ea);
   int tmp = (operand - 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_DEX(int operand, int ea) {
   if (X >= 0) {
      X = (X - 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEY(int operand, int ea) {
   if (Y >= 0) {
      Y = (Y - 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_EOR(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A ^ operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INCA(int operand, int ea) {
   if (A >= 0) {
      A = (A + 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INC(int operand, int ea) {
   memory_read(operand, ea);
   int tmp = (operand + 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_INX(int operand, int ea) {
   if (X >= 0) {
      X = (X + 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_INY(int operand, int ea) {
   if (Y >= 0) {
      Y = (Y + 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_JSR(int operand, int ea) {
   // JSR: the operand is the data pushed to the stack (PCH, PCL)
   if (S >= 0) {
      memory_write((operand >> 8) & 255, 0x100 + S);
      S = (S - 1) & 255;
      memory_write(operand & 255, 0x100 + S);
      S = (S - 1) & 255;
   }
}

static void op_LDA(int operand, int ea) {
   memory_read(operand, ea);
   A = operand;
   if (!MS) {
      B = (operand >> 8) & 0xff;
   }
   set_NZ(A);
}

static void op_LDX(int operand, int ea) {
   memory_read(operand, ea);
   X = operand;
   set_NZ(X);
}

static void op_LDY(int operand, int ea) {
   memory_read(operand, ea);
   Y = operand;
   set_NZ(Y);
}

static void op_LSRA(int operand, int ea) {
   if (A >= 0) {
      C = A & 1;
      A = A >> 1;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_LSR(int operand, int ea) {
   memory_read(operand, ea);
   C = operand & 1;
   int tmp = operand >> 1;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_ORA(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A | operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_PHA(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   op_STA(operand, -1);
}

static void op_PHP(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   check_NVDIZC(operand);
   set_NVDIZC(operand);
}

static void op_PHX(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   op_STX(operand, -1);
}

static void op_PHY(int operand, int ea) {
   if (S >= 0) {
      memory[0x100 + S] = operand;
      S = (S - 1) & 255;
   }
   op_STY(operand, -1);
}

static void op_PLA(int operand, int ea) {
   A = operand;
   set_NZ(A);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLP(int operand, int ea) {
   set_NVDIZC(operand);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLX(int operand, int ea) {
   X = operand;
   set_NZ(X);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLY(int operand, int ea) {
   Y = operand;
   set_NZ(Y);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_ROLA(int operand, int ea) {
   if (A >= 0 && C >= 0) {
      int tmp = (A << 1) + C;
      C = (tmp >> 8) & 1;
      A = tmp & 255;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
}

static void op_ROL(int operand, int ea) {
   if (C >= 0) {
      int tmp = (operand << 1) + C;
      C = (tmp >> 8) & 1;
      tmp = tmp & 255;
      set_NZ(tmp);
      memory_write(tmp, ea);
   } else {
      set_NZC_unknown();
   }
}

static void op_RORA(int operand, int ea) {
   if (A >= 0 && C >= 0) {
      int tmp = (A >> 1) + (C << 7);
      C = A & 1;
      A = tmp;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
}

static void op_ROR(int operand, int ea) {
   if (C >= 0) {
      int tmp = (operand >> 1) + (C << 7);
      C = operand & 1;
      set_NZ(tmp);
      memory_write(tmp, ea);
   } else {
      set_NZC_unknown();
   }
}

static void op_RTS(int operand, int ea) {
   // RTS: the operand is the data pulled from the stack (PCL, PCH)
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);
   }
   // The +1 is handled elsewhere
   PC = operand & 0xffff;
}

static void op_RTI(int operand, int ea) {
   // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
   set_NVDIZC((operand >> 16) & 255);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read((operand >> 16) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);
   }
}

static void op_SBC(int operand, int ea) {
   memory_read(operand, ea);
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
}

static void op_SEC(int operand, int ea) {
   C = 1;
}

static void op_SED(int operand, int ea) {
   D = 1;
}

static void op_SEI(int operand, int ea) {
   I = 1;
}

static void op_STA(int operand, int ea) {
   memory_write(operand, ea);
   if (A >= 0) {
      if (operand != A) {
         failflag = 1;
      }
   }
   A = operand;
}

static void op_STX(int operand, int ea) {
   memory_write(operand, ea);
   if (X >= 0) {
      if (operand != X) {
         failflag = 1;
      }
   }
   X = operand;
}

static void op_STY(int operand, int ea) {
   memory_write(operand, ea);
   if (Y >= 0) {
      if (operand != Y) {
         failflag = 1;
      }
   }
   Y = operand;
}

static void op_STZ(int operand, int ea) {
   memory_write(0, ea);
   if (operand != 0) {
      failflag = 1;
   }
}

static void op_TAX(int operand, int ea) {
   if (A >= 0) {
      X = A;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
}

static void op_TAY(int operand, int ea) {
   if (A >= 0) {
      Y = A;
      set_NZ(Y);
   } else {
      Y = -1;
      set_NZ_unknown();
   }
}

static void op_TSB(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         memory_write(memory[ea] | A, ea);
      }
   } else {
      Z = -1;
   }
}
static void op_TRB(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         memory_write(memory[ea] & ~A, ea);
      }
   } else {
      Z = -1;
   }
}

static void op_TSX(int operand, int ea) {
   if (S >= 0) {
      X = S;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
}

static void op_TXA(int operand, int ea) {
   if (X >= 0) {
      A = X;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
}

static void op_TXS(int operand, int ea) {
   if (X >= 0) {
      S = X;
   } else {
      S = -1;
   }
}

static void op_TYA(int operand, int ea) {
   if (Y >= 0) {
      A = Y;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
}

// ====================================================================
// Opcode Tables
// ====================================================================

static InstrType instr_table_65c816[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, 0, WRITEOP,  op_BRK},
   /* 01 */   { "ORA",  0, INDX  , 6, 0, READOP,   op_ORA},
   /* 02 */   { "COP",  0, IMM   , 7, 0, READOP,   op_COP},
   /* 03 */   { "ORA",  0, SR    , 4, 0, READOP,   op_ORA},
   /* 04 */   { "TSB",  0, ZP    , 5, 0, TSBTRBOP, op_TSB},
   /* 05 */   { "ORA",  0, ZP    , 3, 0, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, 0, RMWOP,    op_ASL},
   /* 07 */   { "ORA",  0, IDL   , 6, 0, READOP,   op_ORA},
   /* 08 */   { "PHP",  0, IMP   , 3, 0, WRITEOP,  op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, 0, READOP,   op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, 0, READOP,   op_ASLA},
   /* 0B */   { "PHD",  0, IMP   , 4, 0, WRITEOP,  op_PHD},
   /* 0C */   { "TSB",  0, ABS   , 6, 0, TSBTRBOP, op_TSB},
   /* 0D */   { "ORA",  0, ABS   , 4, 0, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, 0, RMWOP,    op_ASL},
   /* 0F */   { "ORA",  0, ABL   , 5, 0, READOP,   op_ORA},
   /* 10 */   { "BPL",  0, BRA   , 2, 0, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, 0, READOP,   op_ORA},
   /* 12 */   { "ORA",  0, IND   , 5, 0, READOP,   op_ORA},
   /* 13 */   { "ORA",  0, ISY   , 7, 0, READOP,   op_ORA},
   /* 14 */   { "TRB",  0, ZP    , 5, 0, TSBTRBOP, op_TRB},
   /* 15 */   { "ORA",  0, ZPX   , 4, 0, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, 0, RMWOP,    op_ASL},
   /* 17 */   { "ORA",  0, IDLY  , 6, 0, READOP,   op_ORA},
   /* 18 */   { "CLC",  0, IMP   , 2, 0, READOP,   op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, 0, READOP,   op_ORA},
   /* 1A */   { "INC",  0, IMPA  , 2, 0, READOP,   op_INCA},
   /* 1B */   { "TCS",  0, IMP   , 2, 0, READOP,   op_TCS},
   /* 1C */   { "TRB",  0, ABS   , 6, 0, TSBTRBOP, op_TRB},
   /* 1D */   { "ORA",  0, ABSX  , 4, 0, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 7, 0, RMWOP,    op_ASL},
   /* 1F */   { "ORA",  0, ALX   , 5, 0, READOP,   op_ORA},
   /* 20 */   { "JSR",  0, ABS   , 6, 0, READOP,   op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, 0, READOP,   op_AND},
   /* 22 */   { "JSL",  0, ABL   , 8, 0, READOP,   op_JSL},
   /* 23 */   { "AND",  0, SR    , 4, 0, READOP,   op_AND},
   /* 24 */   { "BIT",  0, ZP    , 3, 0, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, 0, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, 0, RMWOP,    op_ROL},
   /* 27 */   { "AND",  0, IDL   , 6, 0, READOP,   op_AND},
   /* 28 */   { "PLP",  0, IMP   , 4, 0, READOP,   op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, 0, READOP,   op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, 0, READOP,   op_ROLA},
   /* 2B */   { "PLD",  0, IMP   , 5, 0, READOP,   op_PLD},
   /* 2C */   { "BIT",  0, ABS   , 4, 0, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, 0, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, 0, RMWOP,    op_ROL},
   /* 2F */   { "AND",  0, ABL   , 5, 0, READOP,   op_AND},
   /* 30 */   { "BMI",  0, BRA   , 2, 0, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, 0, READOP,   op_AND},
   /* 32 */   { "AND",  0, IND   , 5, 0, READOP,   op_AND},
   /* 33 */   { "AND",  0, ISY   , 7, 0, READOP,   op_AND},
   /* 34 */   { "BIT",  0, ZPX   , 4, 0, READOP,   op_BIT},
   /* 35 */   { "AND",  0, ZPX   , 4, 0, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, 0, RMWOP,    op_ROL},
   /* 37 */   { "AND",  0, IDLY  , 6, 0, READOP,   op_AND},
   /* 38 */   { "SEC",  0, IMP   , 2, 0, READOP,   op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, 0, READOP,   op_AND},
   /* 3A */   { "DEC",  0, IMPA  , 2, 0, READOP,   op_DECA},
   /* 3B */   { "TSC",  0, IMP   , 2, 0, READOP,   op_TSC},
   /* 3C */   { "BIT",  0, ABSX  , 4, 0, READOP,   op_BIT},
   /* 3D */   { "AND",  0, ABSX  , 4, 0, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 7, 0, RMWOP,    op_ROL},
   /* 3F */   { "AND",  0, ALX   , 5, 0, READOP,   op_AND},
   /* 40 */   { "RTI",  0, IMP   , 6, 0, READOP,   op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, 0, READOP,   op_EOR},
   /* 42 */   { "WDM",  0, IMM   , 2, 0, READOP,   0},
   /* 43 */   { "EOR",  0, SR    , 4, 0, READOP,   op_EOR},
   /* 44 */   { "MVP",  0, BM    , 7, 0, WRITEOP,  op_MVP},
   /* 45 */   { "EOR",  0, ZP    , 3, 0, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, 0, RMWOP,    op_LSR},
   /* 47 */   { "EOR",  0, IDL   , 6, 0, READOP,   op_EOR},
   /* 48 */   { "PHA",  0, IMP   , 3, 0, WRITEOP,  op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, 0, READOP,   op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, 0, READOP,   op_LSRA},
   /* 4B */   { "PHK",  0, IMP   , 3, 0, WRITEOP,  op_PHK},
   /* 4C */   { "JMP",  0, ABS   , 3, 0, READOP,   0},
   /* 4D */   { "EOR",  0, ABS   , 4, 0, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, 0, RMWOP,    op_LSR},
   /* 4F */   { "EOR",  0, ABL   , 5, 0, READOP,   op_EOR},
   /* 50 */   { "BVC",  0, BRA   , 2, 0, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, 0, READOP,   op_EOR},
   /* 52 */   { "EOR",  0, IND   , 5, 0, READOP,   op_EOR},
   /* 53 */   { "EOR",  0, ISY   , 7, 0, READOP,   op_EOR},
   /* 54 */   { "MVN",  0, BM    , 7, 0, WRITEOP,  op_MVN},
   /* 55 */   { "EOR",  0, ZPX   , 4, 0, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, 0, RMWOP,    op_LSR},
   /* 57 */   { "EOR",  0, IDLY  , 6, 0, READOP,   op_EOR},
   /* 58 */   { "CLI",  0, IMP   , 2, 0, READOP,   op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, 0, READOP,   op_EOR},
   /* 5A */   { "PHY",  0, IMP   , 3, 0, WRITEOP,  op_PHY},
   /* 5B */   { "TCD",  0, IMP   , 2, 0, READOP,   op_TCD},
   /* 5C */   { "JML",  0, ABL   , 4, 0, READOP,   0},
   /* 5D */   { "EOR",  0, ABSX  , 4, 0, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 7, 0, RMWOP,    op_LSR},
   /* 5F */   { "EOR",  0, ALX   , 5, 0, READOP,   op_EOR},
   /* 60 */   { "RTS",  0, IMP   , 6, 0, READOP,   op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, 1, READOP,   op_ADC},
   /* 62 */   { "PER",  0, IMP   , 6, 0, WRITEOP,  op_PER},
   /* 63 */   { "ADC",  0, SR    , 4, 0, READOP,   op_ADC},
   /* 64 */   { "STZ",  0, ZP    , 3, 0, WRITEOP,  op_STZ},
   /* 65 */   { "ADC",  0, ZP    , 3, 1, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, 0, RMWOP,    op_ROR},
   /* 67 */   { "ADC",  0, IDL   , 6, 0, READOP,   op_ADC},
   /* 68 */   { "PLA",  0, IMP   , 4, 0, READOP,   op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, 1, READOP,   op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, 0, READOP,   op_RORA},
   /* 6B */   { "RTL",  0, IMP   , 6, 0, READOP,   op_RTL},
   /* 6C */   { "JMP",  0, IND16 , 5, 0, READOP,   0},
   /* 6D */   { "ADC",  0, ABS   , 4, 1, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, 0, RMWOP,    op_ROR},
   /* 6F */   { "ADC",  0, ABL   , 5, 0, READOP,   op_ADC},
   /* 70 */   { "BVS",  0, BRA   , 2, 0, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, 1, READOP,   op_ADC},
   /* 72 */   { "ADC",  0, IND   , 5, 1, READOP,   op_ADC},
   /* 73 */   { "ADC",  0, ISY   , 7, 0, READOP,   op_ADC},
   /* 74 */   { "STZ",  0, ZPX   , 4, 0, WRITEOP,  op_STZ},
   /* 75 */   { "ADC",  0, ZPX   , 4, 1, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, 0, RMWOP,    op_ROR},
   /* 77 */   { "ADC",  0, IDLY  , 6, 0, READOP,   op_ADC},
   /* 78 */   { "SEI",  0, IMP   , 2, 0, READOP,   op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, 1, READOP,   op_ADC},
   /* 7A */   { "PLY",  0, IMP   , 4, 0, READOP,   op_PLY},
   /* 7B */   { "TDC",  0, IMP   , 2, 0, READOP,   op_TDC},
   /* 7C */   { "JMP",  0, IND1X , 6, 0, READOP,   0},
   /* 7D */   { "ADC",  0, ABSX  , 4, 1, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 7, 0, RMWOP,    op_ROR},
   /* 7F */   { "ADC",  0, ALX   , 5, 0, READOP,   op_ADC},
   /* 80 */   { "BRA",  0, BRA   , 3, 0, READOP,   0},
   /* 81 */   { "STA",  0, INDX  , 6, 0, WRITEOP,  op_STA},
   /* 82 */   { "BRL",  0, BRL   , 4, 0, READOP,   0},
   /* 83 */   { "STA",  0, SR    , 4, 0, WRITEOP,  op_STA},
   /* 84 */   { "STY",  0, ZP    , 3, 0, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, 0, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, 0, WRITEOP,  op_STX},
   /* 87 */   { "STA" , 0, IDL   , 6, 0, WRITEOP,  op_STA},
   /* 88 */   { "DEY",  0, IMP   , 2, 0, READOP,   op_DEY},
   /* 89 */   { "BIT",  0, IMM   , 2, 0, READOP,   op_BIT_IMM},
   /* 8A */   { "TXA",  0, IMP   , 2, 0, READOP,   op_TXA},
   /* 8B */   { "PHB",  0, IMP   , 3, 0, WRITEOP,  op_PHB},
   /* 8C */   { "STY",  0, ABS   , 4, 0, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, 0, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, 0, WRITEOP,  op_STX},
   /* 8F */   { "STA",  0, ABL   , 5, 0, WRITEOP,  op_STA},
   /* 90 */   { "BCC",  0, BRA   , 2, 0, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, 0, WRITEOP,  op_STA},
   /* 92 */   { "STA",  0, IND   , 5, 0, WRITEOP,  op_STA},
   /* 93 */   { "STA",  0, ISY   , 7, 0, WRITEOP,  op_STA},
   /* 94 */   { "STY",  0, ZPX   , 4, 0, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, 0, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, 0, WRITEOP,  op_STX},
   /* 97 */   { "STA",  0, IDLY  , 6, 0, WRITEOP,  op_STA},
   /* 98 */   { "TYA",  0, IMP   , 2, 0, READOP,   op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, 0, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, 0, READOP,   op_TXS},
   /* 9B */   { "TXY",  0, IMP   , 2, 0, READOP,   op_TXY},
   /* 9C */   { "STZ",  0, ABS   , 4, 0, WRITEOP,  op_STZ},
   /* 9D */   { "STA",  0, ABSX  , 5, 0, WRITEOP,  op_STA},
   /* 9E */   { "STZ",  0, ABSX  , 5, 0, WRITEOP,  op_STZ},
   /* 9F */   { "STA",  0, ALX   , 5, 0, WRITEOP,  op_STA},
   /* A0 */   { "LDY",  0, IMM   , 2, 0, READOP,   op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, 0, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, 0, READOP,   op_LDX},
   /* A3 */   { "LDA",  0, SR    , 4, 0, READOP,   op_LDA},
   /* A4 */   { "LDY",  0, ZP    , 3, 0, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, 0, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, 0, READOP,   op_LDX},
   /* A7 */   { "LDA",  0, IDL   , 6, 0, READOP,   op_LDA},
   /* A8 */   { "TAY",  0, IMP   , 2, 0, READOP,   op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, 0, READOP,   op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, 0, READOP,   op_TAX},
   /* AB */   { "PLB",  0, IMP   , 4, 0, READOP,   op_PLB},
   /* AC */   { "LDY",  0, ABS   , 4, 0, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, 0, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, 0, READOP,   op_LDX},
   /* AF */   { "LDA",  0, ABL   , 5, 0, READOP,   op_LDA},
   /* B0 */   { "BCS",  0, BRA   , 2, 0, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, 0, READOP,   op_LDA},
   /* B2 */   { "LDA",  0, IND   , 5, 0, READOP,   op_LDA},
   /* B3 */   { "LDA",  0, ISY   , 7, 0, READOP,   op_LDA},
   /* B4 */   { "LDY",  0, ZPX   , 4, 0, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, 0, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, 0, READOP,   op_LDX},
   /* B7 */   { "LDA",  0, IDLY  , 6, 0, READOP,   op_LDA},
   /* B8 */   { "CLV",  0, IMP   , 2, 0, READOP,   op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, 0, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, 0, READOP,   op_TSX},
   /* BB */   { "TYX",  0, IMP   , 2, 0, READOP,   op_TYX},
   /* BC */   { "LDY",  0, ABSX  , 4, 0, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, 0, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, 0, READOP,   op_LDX},
   /* BF */   { "LDA",  0, ALX   , 5, 0, READOP,   op_LDA},
   /* C0 */   { "CPY",  0, IMM   , 2, 0, READOP,   op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, 0, READOP,   op_CMP},
   /* C2 */   { "REP",  0, IMM   , 3, 0, READOP,   op_REP},
   /* C3 */   { "CMP",  0, SR    , 4, 0, READOP,   op_CMP},
   /* C4 */   { "CPY",  0, ZP    , 3, 0, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, 0, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, 0, RMWOP,    op_DEC},
   /* C7 */   { "CMP",  0, IDL   , 6, 0, READOP,   op_CMP},
   /* C8 */   { "INY",  0, IMP   , 2, 0, READOP,   op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, 0, READOP,   op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, 0, READOP,   op_DEX},
   /* CB */   { "WAI",  0, IMP   , 1, 0, READOP,   0},        // WD65C02=3
   /* CC */   { "CPY",  0, ABS   , 4, 0, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, 0, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, 0, RMWOP,    op_DEC},
   /* CF */   { "CMP",  0, ABL   , 5, 0, READOP,   op_CMP},
   /* D0 */   { "BNE",  0, BRA   , 2, 0, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, 0, READOP,   op_CMP},
   /* D2 */   { "CMP",  0, IND   , 5, 0, READOP,   op_CMP},
   /* D3 */   { "CMP",  0, ISY   , 7, 0, READOP,   op_CMP},
   /* D4 */   { "PEI",  0, IND   , 6, 0, WRITEOP,  op_PEI},
   /* D5 */   { "CMP",  0, ZPX   , 4, 0, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, 0, RMWOP,    op_DEC},
   /* D7 */   { "CMP" , 0, IDLY  , 6, 0, READOP,   op_CMP},
   /* D8 */   { "CLD",  0, IMP   , 2, 0, READOP,   op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, 0, READOP,   op_CMP},
   /* DA */   { "PHX",  0, IMP   , 3, 0, WRITEOP,  op_PHX},
   /* DB */   { "STP",  0, IMP   , 1, 0, READOP,   0},        // WD65C02=3
   /* DC */   { "JMP",  0, IAL   , 6, 0, READOP,   0},
   /* DD */   { "CMP",  0, ABSX  , 4, 0, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, 0, RMWOP,    op_DEC},
   /* DF */   { "CMP",  0, ALX   , 5, 0, READOP,   op_CMP},
   /* E0 */   { "CPX",  0, IMM   , 2, 0, READOP,   op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, 1, READOP,   op_SBC},
   /* E2 */   { "SEP",  0, IMM   , 3, 0, READOP,   op_SEP},
   /* E3 */   { "SBC",  0, SR    , 4, 0, READOP,   op_SBC},
   /* E4 */   { "CPX",  0, ZP    , 3, 0, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, 1, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, 0, RMWOP,    op_INC},
   /* E7 */   { "SBC",  0, IDL   , 6, 0, READOP,   op_SBC},
   /* E8 */   { "INX",  0, IMP   , 2, 0, READOP,   op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, 1, READOP,   op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, 0, READOP,   0},
   /* EB */   { "XBA",  0, IMP   , 3, 0, READOP,   op_XBA},
   /* EC */   { "CPX",  0, ABS   , 4, 0, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, 1, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, 0, RMWOP,    op_INC},
   /* EF */   { "SBC",  0, ABL   , 5, 0, READOP,   op_SBC},
   /* F0 */   { "BEQ",  0, BRA   , 2, 0, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, 1, READOP,   op_SBC},
   /* F2 */   { "SBC",  0, IND   , 5, 1, READOP,   op_SBC},
   /* F3 */   { "SBC",  0, ISY   , 7, 0, READOP,   op_SBC},
   /* F4 */   { "PEA",  0, ABS   , 5, 0, WRITEOP,  op_PEA},
   /* F5 */   { "SBC",  0, ZPX   , 4, 1, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, 0, RMWOP,    op_INC},
   /* F7 */   { "SBC",  0, IDLY  , 6, 0, READOP,   op_SBC},
   /* F8 */   { "SED",  0, IMP   , 2, 0, READOP,   op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, 1, READOP,   op_SBC},
   /* FA */   { "PLX",  0, IMP   , 4, 0, READOP,   op_PLX},
   /* FB */   { "XCE",  0, IMP   , 2, 0, READOP,   op_XCE},
   /* FC */   { "JSR",  0, IND1X , 8, 0, READOP,   0},
   /* FD */   { "SBC",  0, ABSX  , 4, 1, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, 0, RMWOP,    op_INC},
   /* FF */   { "SBC",  0, ALX   , 5, 0, READOP,   op_SBC}
};
