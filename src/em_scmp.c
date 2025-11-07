#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "memory.h"
#include "em_scmp.h"

// Define this to base JMP taken/not taken on instruction cycle counts
#define JMP_BASED_ON_CYCLES

// ====================================================================
// TODO:
// ====================================================================
//
// Check cycle counts for undefined opcodes

// ====================================================================
// Sample point definitions
// ====================================================================

int clkdiv;

// FPGA:  1; INS8060: 1
#define CYCLE_OPCODE    ( 1 * clkdiv)
// FPGA:  3; INS8060: ?
#define CYCLE_SX        ( 3 * clkdiv)
// FPGA:  5; INS8060: 5
#define CYCLE_OP1       ( 5 * clkdiv)
// FPGA: 15; INS8060: 14
#define CYCLE_READ      (15 * clkdiv)
// FPGA: 15; INS8060: 14
#define CYCLE_WRITE     (15 * clkdiv)
// FPGA: 14; INS8060: 14
#define CYCLE_RMW_READ  (14 * clkdiv)
// FPGA: 17; INS8060: 18
#define CYCLE_RMW_WRITE (18 * clkdiv)

// ====================================================================
// Type Defs
// ====================================================================

typedef enum {
   INH,
   EXT,
   PTR,
   IMM,
   PCREL,
   INDEX,
   AUTO
} AddrMode ;

typedef enum {
   READOP,
   WRITEOP,
   RMWOP,
   JMPOP,
   OTHER
} OpType;

typedef struct {
   int len;
   const char *fmt;
} AddrModeType;

typedef int operand_t;

typedef int ea_t;

typedef struct {
   const char *mnemonic;
   int undocumented;
   AddrMode mode;
   int cycles;
   OpType optype;
   int (*emulate)(operand_t, ea_t, sample_t *);
   int len;
   const char *fmt;
} InstrType;


// ====================================================================
// Static variables
// ====================================================================

#define OFFSET_A    2
#define OFFSET_E    7
#define OFFSET_PX  13
#define OFFSET_CY  45
#define OFFSET_OV  50
#define OFFSET_SB  55
#define OFFSET_SA  60
#define OFFSET_IE  65
#define OFFSET_F2  70
#define OFFSET_F1  75
#define OFFSET_F0  80
#define OFFSET_END 81

#define STEP_PX     8

static const char default_state[] = "A=?? E=?? PC=???? P1=???? P2=???? P3=???? CY=? OV=? SB=? SA=? IE=? F2=? F1=? F0=?";

static InstrType *instr_table;

static AddrModeType addr_mode_table[] = {
   {1,    "%1$s"},                  // INH
   {1,    "%1$s"},                  // E
   {1,    "%1$s P%2$d"},            // PTR
   {2,    "%1$s %2$02X"},           // IMM
   {2,    "%1$s %2$s"},             // PCREL
   {2,    "%1$s %2$d(P%3$d)"},      // INDEX
   {2,    "%1$s @%2$d(P%3$d)"},     // AUTO
};

// SC/MP registers: -1 means unknown
static int A = -1;
static int E = -1;
static int PL[4] = {-1, -1, -1, -1};
static int PH[4] = {-1, -1, -1, -1};

// SC/MP flags: -1 means unknown
static int CY = -1;
static int OV = -1;
static int SB = -1;
static int SA = -1;
static int IE = -1;
static int F2 = -1;
static int F1 = -1;
static int F0 = -1;

static char ILLEGAL[] = "???  ";

// ====================================================================
// Forward declarations
// ====================================================================

static InstrType instr_table_scmp[];

// ====================================================================
// Helper Methods
// ====================================================================

static int get_ptr(int i) {
   if (PL[i] >= 0 && PH[i] >= 0) {
      return (PH[i] << 8) | PL[i];
   } else {
      return -1;
   }
}

static int get_pc() {
   return get_ptr(0);
}

static void set_ptr(int i, int ea) {
   if (ea >= 0) {
      PL[i] = ea & 0xff;
      PH[i] = (ea >> 8) & 0xff;
   } else {
      PL[i] = -1;
      PH[i] = -1;
   }
}

static void set_pc(int ea) {
   set_ptr(0, ea);
}


static void set_flags(int operand) {
   if (operand >= 0) {
      CY = (operand >> 7) & 1;
      OV = (operand >> 6) & 1;
      //      SB = (operand >> 5) & 1;
      //      SA = (operand >> 4) & 1;
      IE = (operand >> 3) & 1;
      F2 = (operand >> 2) & 1;
      F1 = (operand >> 1) & 1;
      F0 = (operand >> 0) & 1;
   } else {
      CY = -1;
      OV = -1;
      //      SB = -1;
      //      SA = -1;
      IE = -1;
      F2 = -1;
      F1 = -1;
      F0 = -1;
   }
}

static int get_flags() {
   if (CY >= 0 && OV >= 0 && SB >= 0 && SA >= 0 && IE >= 0 && F2 >= 0 && F1 >= 0 && F0 >= 0) {
      return (CY << 7) | (OV << 6) | (SB << 5) | (SA << 4) | (IE << 3) | (F2 << 2) | (F1 << 1) | F0;
   } else {
      return -1;
   }
}


static void interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction, int pc_offset) {
   IE = 0;
   PL[0] = PL[3];
   PH[0] = PH[3];
}


static int get_num_cycles(sample_t *sample_q, int intr_seen) {
   // Force opcode to XPPC P3 (3F) if interrupt detected
   int opcode = intr_seen ? 0x3F : sample_q[CYCLE_OPCODE].data;
   InstrType *instr = &instr_table[opcode];
   int cycle_count = instr->cycles;
   if (opcode == 0x8F) {
      // DLY, increase by A*2 + D*514
      if (A >= 0) {
         int op1 = sample_q[CYCLE_OP1].data;
         cycle_count += 2 * A + 514 * op1;
      } else {
         cycle_count = -1;
      }
   } else if (instr->optype == JMPOP) {
      int type = opcode & 0x0C;
      if (type) {
         if (A >= 0) {
            switch (type) {
            case 0x04:
               // JP not taken, reduce by 2
               if (A >= 0x80) {
                  cycle_count -= 2;
               }
               break;
            case 0x08:
               // JZ not taken, reduce by 2
               if (A > 0x00) {
                  cycle_count -= 2;
               }
               break;
            case 0x0C:
               // JNZ not taken, reduce by 2
               if (A == 0x00) {
                  cycle_count -= 2;
               }
               break;
            }
         } else {
            cycle_count = -1;
         }
      }
   }
   if (cycle_count >= 0) {
      cycle_count *= clkdiv;
   }
   return cycle_count;
}

static int count_cycles_without_ads(sample_t *sample_q, int num_samples, int intr_seen) {
   int start = 1;
   // The instruction should start with ADS cycle that precedes the
   // next opcode fetch. The upper nibble should be 3 and the lower
   // nibble A[15:12]
   int mask  = 0xf0;
   int value = 0x30;
   if (PH[0] >= 0) {
      mask  |= 0x0F;
      value |= PH[0] >> 4;
   }
   if ((sample_q[0].data & mask) == value) {
      int num_cycles = get_num_cycles(sample_q, intr_seen);
      if (num_cycles >= 0) {
         return num_cycles;
      }
      printf("cycle prediction unknown\n");
      start = 4 * clkdiv;
   } else {
      printf("Warning: incorrect data in opcode fetch ADS cycle: 0x%02X & 0x%02X != 0x%02X, resyncing using heuristic\n", sample_q[0].data, mask, value);
   }
   for (int i = start; i < num_samples; i++) {
      if ((sample_q[i].data & mask) == value) {
         return i;
      }
   }
   printf("no opcode fetch signature found\n");
   return num_samples;
}

static int count_cycles_with_ads(sample_t *sample_q, int num_samples, int intr_seen) {
   if (sample_q[0].type == OPCODE) {
      // Synced to the instruction stream
      for (int i = 1; i < num_samples; i++) {
         if (sample_q[i].type == LAST) {
            return 0;
         }
         if (sample_q[i].type == OPCODE) {
            // Validate the num_cycles passed in
            int expected = get_num_cycles(sample_q, intr_seen);
            if (expected >= 0) {
               if (i != expected) {
                  printf ("opcode %02x: cycle prediction fail: expected %d actual %d\n", sample_q[0].data, expected, i);
               }
            }
            return i;
         }
      }
   } else {
      // Not synced to the instruction stream
      for (int i = 1; i < num_samples; i++) {
         // Search for the start of the next instruction
         if (sample_q[i].type == OPCODE) {
            return i;
         } else if (sample_q[i].type == LAST) {
            return 0;
         }
      }
   }
   return 1;
}

// ====================================================================
// Public Methods
// ====================================================================

static void em_scmp_init(arguments_t *args) {

   instr_table = instr_table_scmp;

   clkdiv = args->clkdiv;

   // Optional initialization of status register
   if (args->psr_reg >= 0) {
      CY = (args->psr_reg >> 7) & 1;
      OV = (args->psr_reg >> 6) & 1;
      SB = (args->psr_reg >> 5) & 1;
      SA = (args->psr_reg >> 4) & 1;
      IE = (args->psr_reg >> 3) & 1;
      F2 = (args->psr_reg >> 2) & 1;
      F1 = (args->psr_reg >> 1) & 1;
      F0 = (args->psr_reg >> 0) & 1;
   }

   InstrType *instr = instr_table;
   for (int i = 0; i < 256; i++) {
      // Remove the undocumented instructions, if not supported
      if (instr->undocumented && !args->undocumented) {
         instr->mnemonic = ILLEGAL;
         instr->mode     = INH;
         instr->cycles   = 1;
      }
      // Copy the length and format from the address mode, for efficiency
      instr->len = addr_mode_table[instr->mode].len;
      instr->fmt = addr_mode_table[instr->mode].fmt;
      instr++;
   }
}

static int em_scmp_match_interrupt(sample_t *sample_q, int num_samples) {
   return (IE == 1 && SA == 1);
}

static int em_scmp_count_cycles(sample_t *sample_q, int num_samples, int intr_seen) {
   if (sample_q[0].type == UNKNOWN) {
      return count_cycles_without_ads(sample_q, num_samples, intr_seen);
   } else {
      return count_cycles_with_ads(sample_q, num_samples, intr_seen);
   }
}

static void em_scmp_reset(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   instruction->pc = -1;
   A = 0;
   E = 0;
   for (int i = 0; i <= 3; i++) {
      PL[i] = 0;
      PH[i] = 0;
   }
   CY = 0;
   OV = 0;
   SB = 1; // Default to seeimg a floating high value
   SA = 1; // Default to seeimg a floating high value
   IE = 0;
   F2 = 0;
   F1 = 0;
   F0 = 0;
}

static void em_scmp_interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   interrupt(sample_q, num_cycles, instruction, 0);
}

static void em_scmp_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction) {

   // Unpack the instruction bytes
   int opcode = sample_q[CYCLE_OPCODE].data;

   // Update SA and SB during fetch
   if (sample_q[CYCLE_SX].sa >= 0) {
      SA = sample_q[CYCLE_SX].sa;
   }
   if (sample_q[CYCLE_SX].sb >= 0) {
      SB = sample_q[CYCLE_SX].sb;
   }

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];
   int opcount = instr->len - 1;
   int op1 = (opcount < 1) ? 0 : sample_q[CYCLE_OP1].data;
   int pc = get_pc();

   if (pc >= 0) {
      pc = (pc & 0xf000) | ((pc + 1) & 0xfff);
      memory_read(opcode, pc, MEM_FETCH);
   }

   // Save the instruction state
   instruction->pc = pc;
   instruction->opcode  = opcode;
   instruction->op1     = op1;
   instruction->opcount = opcount;

   // Memory Modelling: Instruction fetches
   if (pc >= 0) {
      if (opcount >= 1) {
         pc = (pc & 0xf000) | ((pc + 1) & 0xfff);
         memory_read(op1, pc, MEM_INSTR);
      }
   }

   // Update pc
   set_pc(pc);

   if (instr->emulate) {

      // For instructions that read or write memory, we need to work out the effective address
      // A displacement of -128 is replaced by the E register
      // Note: this should exclude JMP, ILD, DLD, see:
      // https://www.vintage-radio.net/forum/showthread.php?p=1720142#post1720142
      if ((instr->optype == READOP || instr->optype == WRITEOP) && (instr->mode == PCREL || instr->mode == INDEX || instr->mode == AUTO)) {
         if (((int8_t) op1) == -128) {
            op1 = E;
         }
      }

      // Compute the effective address
      int ea = -1;
      if (op1 >= 0) {
         switch (instr->mode) {
         case PCREL:
         case INDEX:
            ea = get_ptr(opcode & 3);
            if (ea >= 0) {
               ea = (ea & 0xF000) | ((ea + (int8_t)op1) & 0x0FFF);
            }
            break;
         case AUTO:
            ea = get_ptr(opcode & 3);
            if (ea >= 0) {
               int newPtr = (ea & 0xF000) | ((ea + (int8_t)op1) & 0x0FFF);
               if (((int8_t) op1) < 0) {
                  ea = newPtr;
               }
               set_ptr(opcode & 3, newPtr);
            }
            break;
         default:
            break;
         }
      } else {
         if (instr->mode == AUTO) {
            set_ptr(opcode & 3, -1);
         }
      }

      operand_t operand = -1;
      operand_t operand2 = -1;
      if (instr->mode == EXT) {
         operand = E;
      } else if (instr->mode == IMM) {
         operand = op1;
      } else if (instr->mode == PTR) {
         // XPAL, XPAH, XPPC
         operand = opcode & 3;
      } else if (instr->optype == READOP) {
         // LD, AND, OR, XOR, ADD, CAD, DAD
         operand = sample_q[CYCLE_READ].data;
      } else if (instr->optype == WRITEOP) {
         // ST
         operand = sample_q[CYCLE_WRITE].data;
         operand2 = operand;
      } else if (instr->optype == RMWOP) {
         // ILD/DLD
         operand = sample_q[CYCLE_RMW_READ].data;
         operand2 = sample_q[CYCLE_RMW_WRITE].data;
      } else if (instr->optype == JMPOP) {
         // JMP operand is whether JMP was taken or not
         operand = (num_cycles == 11 * clkdiv);
      }

      // Model memory reads
      if (ea >= 0 && (instr->optype == READOP || instr->optype == RMWOP)) {
         memory_read(operand, ea, MEM_DATA);
      }

      // Execute the instruction specific function
      // (This returns -1 if the result is unknown or invalid
      int result = instr->emulate(operand, ea, sample_q);

      if (instr->optype == WRITEOP || instr->optype == RMWOP) {

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
}

static int em_scmp_disassemble(char *buffer, instruction_t *instruction) {

   int numchars = 0;
   int offset;
   char target[16];

   // Unpack the instruction bytes
   int opcode = instruction->opcode;
   int op1    = instruction->op1;
   int pc     = instruction->pc;

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   const char *mnemonic = instr->mnemonic;
   const char *fmt = instr->fmt;
   switch (instr->mode) {
   case INH:
   case EXT:
      numchars = sprintf(buffer, fmt, mnemonic);
      break;
   case PTR:
      numchars = sprintf(buffer, fmt, mnemonic, opcode & 3);
      break;
   case IMM:
      numchars = sprintf(buffer, fmt, mnemonic, op1);
      break;
   case PCREL:
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
   case INDEX:
   case AUTO:
      numchars = sprintf(buffer, fmt, mnemonic, (int)((int8_t)op1), opcode & 3);
      break;
   }
   return numchars;
}

static int em_scmp_get_PC() {
   int pc = get_pc();
   return (pc & 0xf000) | ((pc + 1) & 0xfff);
}


static int em_scmp_get_PB() {
   return 0;
}

static int em_scmp_read_memory(int address) {
   return memory_read_raw(address);
}

static char *em_scmp_get_state(char *buffer) {
   strcpy(buffer, default_state);
   if (A >= 0) {
      write_hex2(buffer + OFFSET_A, A);
   }
   if (E >= 0) {
      write_hex2(buffer + OFFSET_E, E);
   }
   for (int i = 0; i <= 3; i++) {
      if (PH[i] >= 0) {
         write_hex2(buffer + OFFSET_PX + i * STEP_PX, PH[i]);
      }
      if (PL[i] >= 0) {
         write_hex2(buffer + OFFSET_PX + i * STEP_PX + 2, PL[i]);
      }
   }
   if (CY >= 0) {
      buffer[OFFSET_CY] = '0' + CY;
   }
   if (OV >= 0) {
      buffer[OFFSET_OV] = '0' + OV;
   }
   if (SB >= 0) {
      buffer[OFFSET_SB] = '0' + SB;
   }
   if (SA >= 0) {
      buffer[OFFSET_SA] = '0' + SA;
   }
   if (IE >= 0) {
      buffer[OFFSET_IE] = '0' + IE;
   }
   if (F2 >= 0) {
      buffer[OFFSET_F2] = '0' + F2;
   }
   if (F1 >= 0) {
      buffer[OFFSET_F1] = '0' + F1;
   }
   if (F2 >= 0) {
      buffer[OFFSET_F0] = '0' + F0;
   }
   return buffer + OFFSET_END;
}

static int em_scmp_get_and_clear_fail() {
   int ret = failflag;
   failflag = 0;
   return ret;
}

cpu_emulator_t em_scmp = {
   .init = em_scmp_init,
   .match_interrupt = em_scmp_match_interrupt,
   .count_cycles = em_scmp_count_cycles,
   .reset = em_scmp_reset,
   .interrupt = em_scmp_interrupt,
   .emulate = em_scmp_emulate,
   .disassemble = em_scmp_disassemble,
   .get_PC = em_scmp_get_PC,
   .get_PB = em_scmp_get_PB,
   .read_memory = em_scmp_read_memory,
   .get_state = em_scmp_get_state,
   .get_and_clear_fail = em_scmp_get_and_clear_fail
};

// ====================================================================
// Instruction helpers
// ====================================================================

static int bin_add_helper(int val, int operand, int carry) {
   if (val >= 0 && operand >= 0 && carry >= 0) {
      int tmp = val + operand + carry;
      CY = (tmp >> 8) & 1;
      OV = (((val ^ operand) & 0x80) == 0) && (((val ^ tmp) & 0x80) != 0);
      val = tmp & 0xFF;
   } else {
      val = -1;
      CY = -1;
      OV = -1;
   }
   return val;
}

// This matches how we think the INS8060 is implemented
static int dec_add_helper(int val, int operand, int carry) {
   if (val >= 0 && operand >= 0 && carry >= 0) {

      // Add 1
      int lo = (operand & 0x0f) + 0x06;
      int c2 = lo >> 4;
      lo &= 0x0f;

      // Add 2
      lo = (val & 0x0f) + lo + CY;
      int co = lo >> 4;
      lo &= 0x0f;

      // Add 3
      if (!co) {
         lo += 0x0a;
         lo &= 0x0f;
      }

      // Add 4
      int hi = (operand >> 4) + 0x06 + c2;
      hi &= 0x0f;

      // Add 5
      hi = (val >> 4) + hi + co;
      co = hi >> 4;
      hi &= 0x0f;

      // Add 6
      if (!co) {
         hi += 0x0a;
         hi &= 0x0f;
      }

      CY = co;
      val = (hi << 4) + lo;
   } else {
      val = -1;
      CY = -1;
   }
   return val;
}

// ====================================================================
// Individual Instructions
// ====================================================================

static int op_ADD(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = bin_add_helper(A, operand, CY);
   return -1;
}

static int op_AND(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0 && operand >= 0) {
      A = A & operand;
   } else if (A == 0 || operand == 0) {
      A = 0;
   } else {
      A = -1;
   }
   return -1;
}

static int op_CAD(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (operand >= 0) {
      operand = operand ^ 0xFF;
   }
   A = bin_add_helper(A, operand, CY);
   return -1;
}

static int op_CAS(operand_t operand, ea_t ea, sample_t *sample_q) {
   set_flags(A);
   return -1;
}

static int op_CCL(operand_t operand, ea_t ea, sample_t *sample_q) {
   CY = 0;
   return -1;
}

static int op_CSA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = get_flags();
   return -1;
}

static int op_DAD(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = dec_add_helper(A, operand, CY);
   return -1;
}

static int op_DINT(operand_t operand, ea_t ea, sample_t *sample_q) {
   IE = 0;
   return -1;
}

static int op_DLD(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = (operand - 1) & 0xFF;
   return A;
}

static int op_DLY(operand_t operand, ea_t ea, sample_t *sample_q) {
   return -1;
}

static int op_HALT(operand_t operand, ea_t ea, sample_t *sample_q) {
   return -1;
}

static int op_IEN(operand_t operand, ea_t ea, sample_t *sample_q) {
   IE = 1;
   return -1;
}


static int op_ILD(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = (operand + 1) & 0xFF;
   return A;
}

static int op_JMP(operand_t operand, ea_t ea, sample_t *sample_q) {
   set_pc(ea);
   return -1;
}

static int op_JNZ(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if A non zero
#ifdef JMP_BASED_ON_CYCLES
   if (operand) {
      set_pc(ea);
      if (A == 0) {
         failflag = 1;
      }
   } else {
      if (A > 0) {
         failflag = 1;
      }
      // Infer A=0
      A = 0;
   }
#else
   if (A < 0) {
      set_pc(-1);
   } else if (A > 0) {
      set_pc(ea);
   }
#endif
   return -1;
}

static int op_JP(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if positive
#ifdef JMP_BASED_ON_CYCLES
   if (operand) {
      set_pc(ea);
      if (A >= 128) {
         failflag = 1;
      }
   } else {
      if (A >= 0 && A < 128) {
         failflag = 1;
      }
   }
#else
   if (A < 0) {
      set_pc(-1);
   } else if ((A & 0x80) == 0) {
      set_pc(ea);
   }
#endif
   return -1;
}

static int op_JZ(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if A zero
#ifdef JMP_BASED_ON_CYCLES
   if (operand) {
      set_pc(ea);
      if (A > 0) {
         failflag = 1;
      }
      // Infer A=0
      A = 0;
   } else {
      if (A == 0) {
         failflag = 1;
      }
   }
#else
   if (A < 0) {
      set_pc(-1);
   } else if (A == 0) {
      set_pc(ea);
   }
#endif
   return -1;
}

static int op_LD(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = operand;
   return -1;
}

static int op_NOP(operand_t operand, ea_t ea, sample_t *sample_q) {
   return -1;
}

static int op_OR(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >=0 && operand >= 0) {
      A = A | operand;
   } else {
      A = -1;
   }
   return -1;
}

static int op_RR(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0) {
      A = (A >> 1) | ((A & 1) << 7);
   }
   return -1;
}

static int op_RRL(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0 && CY >= 0) {
      int tmpCY = CY;
      CY = A & 1;
      A = (A >> 1) | (tmpCY << 7);
   } else {
      A = -1;
      CY = -1;
   }
   return -1;
}

static int op_SCL(operand_t operand, ea_t ea, sample_t *sample_q) {
   CY = 1;
   return -1;
}

static int op_SIO(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (E >= 0 && sample_q[CYCLE_SX].sin >= 0) {
      E = (sample_q[CYCLE_SX].sin << 7) | (E >> 1);
   }
   return -1;
}

static int op_SR(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0) {
      A = A >> 1;
   }
   return -1;
}

static int op_SRL(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0 && CY >= 0) {
      A = (A >> 1) | (CY << 7);
   } else {
      A = -1;
   }
   return -1;
}

static int op_ST(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0) {
      if (operand != A) {
         failflag = 1;
      }
   }
   A = operand;
   return operand;
}

static int op_XAE(operand_t operand, ea_t ea, sample_t *sample_q) {
   int tmp = A;
   A = E;
   E = tmp;
   return -1;
}

static int op_XOR(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >=0 && operand >= 0) {
      A = A ^ operand;
   } else {
      A = -1;
   }
   return -1;
}

static int op_XPAH(operand_t operand, ea_t ea, sample_t *sample_q) {
   int tmpH = PH[operand];
   PH[operand] = A;
   A = tmpH;
   return -1;
}

static int op_XPAL(operand_t operand, ea_t ea, sample_t *sample_q) {
   int tmpL = PL[operand];
   PL[operand] = A;
   A = tmpL;
   return -1;
}

static int op_XPPC(operand_t operand, ea_t ea, sample_t *sample_q) {
   int tmpL = PL[operand];
   int tmpH = PH[operand];
   PL[operand] = PL[0];
   PH[operand] = PH[0];
   PL[0] = tmpL;
   PH[0] = tmpH;
   return -1;
}

// ====================================================================
// Opcode Tables
// ====================================================================

static InstrType instr_table_scmp[] = {
   /* 00 */   { "HALT ", 0,   INH,  8,     OTHER, op_HALT},
   /* 01 */   { "XAE  ", 0,   INH,  7,     OTHER, op_XAE },
   /* 02 */   { "CCL  ", 0,   INH,  5,     OTHER, op_CCL },
   /* 03 */   { "SCL  ", 0,   INH,  5,     OTHER, op_SCL },
   /* 04 */   { "DINT ", 0,   INH,  6,     OTHER, op_DINT},
   /* 05 */   { "IEN  ", 0,   INH,  6,     OTHER, op_IEN },
   /* 06 */   { "CSA  ", 0,   INH,  5,     OTHER, op_CSA },
   /* 07 */   { "CAS  ", 0,   INH,  6,     OTHER, op_CAS },
   /* 08 */   { "NOP  ", 0,   INH,  5,     OTHER, op_NOP },
   /* 09 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 0F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 10 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 11 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 12 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 13 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 14 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 15 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 16 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 17 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 18 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 19 */   { "SIO  ", 0,   INH,  5,     OTHER, op_SIO },
   /* 1A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 1B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 1C */   { "SR   ", 0,   INH,  5,     OTHER, op_SR  },
   /* 1D */   { "SRL  ", 0,   INH,  5,     OTHER, op_SRL },
   /* 1E */   { "RR   ", 0,   INH,  5,     OTHER, op_RR  },
   /* 1F */   { "RRL  ", 0,   INH,  5,     OTHER, op_RRL },

   /* 20 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 21 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 22 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 23 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 24 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 25 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 26 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 27 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 28 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 29 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 2F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 30 */   { "XPAL ", 0,   PTR,  8,     OTHER, op_XPAL},
   /* 31 */   { "XPAL ", 0,   PTR,  8,     OTHER, op_XPAL},
   /* 32 */   { "XPAL ", 0,   PTR,  8,     OTHER, op_XPAL},
   /* 33 */   { "XPAL ", 0,   PTR,  8,     OTHER, op_XPAL},
   /* 34 */   { "XPAH ", 0,   PTR,  8,     OTHER, op_XPAH},
   /* 35 */   { "XPAH ", 0,   PTR,  8,     OTHER, op_XPAH},
   /* 36 */   { "XPAH ", 0,   PTR,  8,     OTHER, op_XPAH},
   /* 37 */   { "XPAH ", 0,   PTR,  8,     OTHER, op_XPAH},
   /* 38 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 39 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 3A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 3B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 3C */   { "XPPC ", 0,   PTR,  7,     OTHER, op_XPPC},
   /* 3D */   { "XPPC ", 0,   PTR,  7,     OTHER, op_XPPC},
   /* 3E */   { "XPPC ", 0,   PTR,  7,     OTHER, op_XPPC},
   /* 3F */   { "XPPC ", 0,   PTR,  7,     OTHER, op_XPPC},

   /* 40 */   { "LDE  ", 0,   EXT,  6,     OTHER,  op_LD },
   /* 41 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 42 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 43 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 44 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 45 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 46 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 47 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 48 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 49 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 4F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 50 */   { "ANE  ", 0,   EXT,  6,     OTHER, op_AND },
   /* 51 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 52 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 53 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 54 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 55 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 56 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 57 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 58 */   { "ORE  ", 0,   EXT,  6,     OTHER,  op_OR },
   /* 59 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 5F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 60 */   { "XRE  ", 0,   EXT,  6,     OTHER, op_XOR },
   /* 61 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 62 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 63 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 64 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 65 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 66 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 67 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 68 */   { "DAE  ", 0,   EXT, 11,     OTHER, op_DAD },
   /* 69 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 6F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 70 */   { "ADE  ", 0,   EXT,  7,     OTHER, op_ADD },
   /* 71 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 72 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 73 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 74 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 75 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 76 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 77 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 78 */   { "CAE  ", 0,   EXT,  8,     OTHER, op_CAD },
   /* 79 */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7A */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7B */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7C */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7D */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7E */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },
   /* 7F */   { "NOP  ", 1,   INH,  5,     OTHER,      0 },

   /* 80 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 81 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 82 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 83 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 84 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 85 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 86 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 87 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 88 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 89 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8A */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8B */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8C */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8D */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8E */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* 8F */   { "DLY  ", 0,   IMM, 13,     OTHER, op_DLY },

   /* 90 */   { "JMP  ", 0, PCREL, 11,     JMPOP, op_JMP },
   /* 91 */   { "JMP  ", 0, INDEX, 11,     JMPOP, op_JMP },
   /* 92 */   { "JMP  ", 0, INDEX, 11,     JMPOP, op_JMP },
   /* 93 */   { "JMP  ", 0, INDEX, 11,     JMPOP, op_JMP },
   /* 94 */   { "JP   ", 0, PCREL, 11,     JMPOP, op_JP  },
   /* 95 */   { "JP   ", 0, INDEX, 11,     JMPOP, op_JP  },
   /* 96 */   { "JP   ", 0, INDEX, 11,     JMPOP, op_JP  },
   /* 97 */   { "JP   ", 0, INDEX, 11,     JMPOP, op_JP  },
   /* 98 */   { "JZ   ", 0, PCREL, 11,     JMPOP, op_JZ  },
   /* 99 */   { "JZ   ", 0, INDEX, 11,     JMPOP, op_JZ  },
   /* 9A */   { "JZ   ", 0, INDEX, 11,     JMPOP, op_JZ  },
   /* 9B */   { "JZ   ", 0, INDEX, 11,     JMPOP, op_JZ  },
   /* 9C */   { "JNZ  ", 0, PCREL, 11,     JMPOP, op_JNZ },
   /* 9D */   { "JNZ  ", 0, INDEX, 11,     JMPOP, op_JNZ },
   /* 9E */   { "JNZ  ", 0, INDEX, 11,     JMPOP, op_JNZ },
   /* 9F */   { "JNZ  ", 0, INDEX, 11,     JMPOP, op_JNZ },

   /* A0 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A1 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A2 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A3 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A4 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A5 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A6 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A7 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* A8 */   { "ILD  ", 0, INDEX, 22,     RMWOP, op_ILD },
   /* A9 */   { "ILD  ", 0, INDEX, 22,     RMWOP, op_ILD },
   /* AA */   { "ILD  ", 0, INDEX, 22,     RMWOP, op_ILD },
   /* AB */   { "ILD  ", 0, INDEX, 22,     RMWOP, op_ILD },
   /* AC */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* AD */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* AE */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* AF */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },

   /* B0 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B1 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B2 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B3 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B4 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B5 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B6 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B7 */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* B8 */   { "DLD  ", 0, INDEX, 22,     RMWOP, op_DLD },
   /* B9 */   { "DLD  ", 0, INDEX, 22,     RMWOP, op_DLD },
   /* BA */   { "DLD  ", 0, INDEX, 22,     RMWOP, op_DLD },
   /* BB */   { "DLD  ", 0, INDEX, 22,     RMWOP, op_DLD },
   /* BC */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* BD */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* BE */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* BF */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },

   /* C0 */   { "LD   ", 0, INDEX, 18,    READOP, op_LD  },
   /* C1 */   { "LD   ", 0, INDEX, 18,    READOP, op_LD  },
   /* C2 */   { "LD   ", 0, INDEX, 18,    READOP, op_LD  },
   /* C3 */   { "LD   ", 0, INDEX, 18,    READOP, op_LD  },
   /* C4 */   { "LDI  ", 0,   IMM, 10,     OTHER, op_LD  },
   /* C5 */   { "LD   ", 0,  AUTO, 18,    READOP, op_LD  },
   /* C6 */   { "LD   ", 0,  AUTO, 18,    READOP, op_LD  },
   /* C7 */   { "LD   ", 0,  AUTO, 18,    READOP, op_LD  },
   /* C8 */   { "ST   ", 0, INDEX, 18,   WRITEOP, op_ST  },
   /* C9 */   { "ST   ", 0, INDEX, 18,   WRITEOP, op_ST  },
   /* CA */   { "ST   ", 0, INDEX, 18,   WRITEOP, op_ST  },
   /* CB */   { "ST   ", 0, INDEX, 18,   WRITEOP, op_ST  },
   /* CC */   { "NOP  ", 1,   IMM,  9,     OTHER,      0 },
   /* CD */   { "ST   ", 0,  AUTO, 18,   WRITEOP, op_ST  },
   /* CE */   { "ST   ", 0,  AUTO, 18,   WRITEOP, op_ST  },
   /* CF */   { "ST   ", 0,  AUTO, 18,   WRITEOP, op_ST  },

   /* D0 */   { "AND  ", 0, INDEX, 18,    READOP, op_AND },
   /* D1 */   { "AND  ", 0, INDEX, 18,    READOP, op_AND },
   /* D2 */   { "AND  ", 0, INDEX, 18,    READOP, op_AND },
   /* D3 */   { "AND  ", 0, INDEX, 18,    READOP, op_AND },
   /* D4 */   { "ANI  ", 0,   IMM, 10,     OTHER, op_AND },
   /* D5 */   { "AND  ", 0,  AUTO, 18,    READOP, op_AND },
   /* D6 */   { "AND  ", 0,  AUTO, 18,    READOP, op_AND },
   /* D7 */   { "AND  ", 0,  AUTO, 18,    READOP, op_AND },
   /* D8 */   { "OR   ", 0, INDEX, 18,    READOP, op_OR  },
   /* D9 */   { "OR   ", 0, INDEX, 18,    READOP, op_OR  },
   /* DA */   { "OR   ", 0, INDEX, 18,    READOP, op_OR  },
   /* DB */   { "OR   ", 0, INDEX, 18,    READOP, op_OR  },
   /* DC */   { "ORI  ", 0,   IMM, 10,     OTHER, op_OR  },
   /* DD */   { "OR   ", 0,  AUTO, 18,    READOP, op_OR  },
   /* DE */   { "OR   ", 0,  AUTO, 18,    READOP, op_OR  },
   /* DF */   { "OR   ", 0,  AUTO, 18,    READOP, op_OR  },

   /* E0 */   { "XOR  ", 0, INDEX, 18,    READOP, op_XOR },
   /* E1 */   { "XOR  ", 0, INDEX, 18,    READOP, op_XOR },
   /* E2 */   { "XOR  ", 0, INDEX, 18,    READOP, op_XOR },
   /* E3 */   { "XOR  ", 0, INDEX, 18,    READOP, op_XOR },
   /* E4 */   { "XRI  ", 0,   IMM, 10,     OTHER, op_XOR },
   /* E5 */   { "XOR  ", 0,  AUTO, 18,    READOP, op_XOR },
   /* E6 */   { "XOR  ", 0,  AUTO, 18,    READOP, op_XOR },
   /* E7 */   { "XOR  ", 0,  AUTO, 18,    READOP, op_XOR },
   /* E8 */   { "DAD  ", 0, INDEX, 23,    READOP, op_DAD },
   /* E9 */   { "DAD  ", 0, INDEX, 23,    READOP, op_DAD },
   /* EA */   { "DAD  ", 0, INDEX, 23,    READOP, op_DAD },
   /* EB */   { "DAD  ", 0, INDEX, 23,    READOP, op_DAD },
   /* EC */   { "DAI  ", 0,   IMM, 15,     OTHER, op_DAD },
   /* ED */   { "DAD  ", 0,  AUTO, 23,    READOP, op_DAD },
   /* EE */   { "DAD  ", 0,  AUTO, 23,    READOP, op_DAD },
   /* EF */   { "DAD  ", 0,  AUTO, 23,    READOP, op_DAD },

   /* F0 */   { "ADD  ", 0, INDEX, 19,    READOP, op_ADD },
   /* F1 */   { "ADD  ", 0, INDEX, 19,    READOP, op_ADD },
   /* F2 */   { "ADD  ", 0, INDEX, 19,    READOP, op_ADD },
   /* F3 */   { "ADD  ", 0, INDEX, 19,    READOP, op_ADD },
   /* F4 */   { "ADI  ", 0,   IMM, 11,     OTHER, op_ADD },
   /* F5 */   { "ADD  ", 0,  AUTO, 19,    READOP, op_ADD },
   /* F6 */   { "ADD  ", 0,  AUTO, 19,    READOP, op_ADD },
   /* F7 */   { "ADD  ", 0,  AUTO, 19,    READOP, op_ADD },
   /* F8 */   { "CAD  ", 0, INDEX, 20,    READOP, op_CAD },
   /* F9 */   { "CAD  ", 0, INDEX, 20,    READOP, op_CAD },
   /* FA */   { "CAD  ", 0, INDEX, 20,    READOP, op_CAD },
   /* FB */   { "CAD  ", 0, INDEX, 20,    READOP, op_CAD },
   /* FC */   { "CAI  ", 0,   IMM, 12,     OTHER, op_CAD },
   /* FD */   { "CAD  ", 0,  AUTO, 20,    READOP, op_CAD },
   /* FE */   { "CAD  ", 0,  AUTO, 20,    READOP, op_CAD },
   /* FF */   { "CAD  ", 0,  AUTO, 20,    READOP, op_CAD },
};
