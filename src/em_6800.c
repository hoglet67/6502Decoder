#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "memory.h"
#include "em_6800.h"

// ====================================================================
// TODO:
// ====================================================================
//
// Correctly handle WAI (as the CPU halts until interrupt)
// Explore undocumented instructions
// Test DAA
// Tighten match_interrupt() test

// ====================================================================
// Type Defs
// ====================================================================

typedef enum {
   INH,
   ACC,
   IMM8,
   IMM16,
   DIR8,
   DIR16,
   EXT8,
   EXT16,
   IDX8,
   IDX16,
   REL
} AddrMode ;

typedef enum {
   READOP,
   WRITEOP,
   RMWOP,
   BRANCHOP,
   JSRJMPOP,
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
#define OFFSET_B    7
#define OFFSET_X   12
#define OFFSET_S   20
#define OFFSET_H   27
#define OFFSET_I   31
#define OFFSET_N   35
#define OFFSET_Z   39
#define OFFSET_V   43
#define OFFSET_C   47
#define OFFSET_END 48

static const char default_state[] = "A=?? B=?? X=???? SP=???? H=? I=? N=? Z=? V=? C=?";

static InstrType *instr_table;

static AddrModeType addr_mode_table[] = {
   {1,    "%1$s"},                  // IMP
   {1,    "%1$s"},                  // ACC
   {2,    "%1$s #%2$02X"},          // IMM8
   {3,    "%1$s #%2$02X%3$02X"},    // IMM16
   {2,    "%1$s #%2$02X"},          // DIR8
   {2,    "%1$s #%2$02X"},          // DIR16
   {3,    "%1$s %2$02X%3$02X"},     // EXT8
   {3,    "%1$s %2$02X%3$02X"},     // EXT16
   {2,    "%1$s %2$02X,X"},         // IDX8
   {2,    "%1$s %2$02X,X"},         // IDX16
   {2,    "%1$s %2$s"}              // REL
};

// 6800 registers: -1 means unknown
static int A = -1;
static int B = -1;
static int X = -1;
static int S = -1;
static int PC = -1;

// 6800 flags: -1 means unknown
static int H = -1;
static int I = -1;
static int N = -1;
static int Z = -1;
static int V = -1;
static int C = -1;

static char ILLEGAL[] = "???  ";

// BSR/JSR return address cycle positions
static int bsr_rel_pcl = 3;
static int bsr_rel_pch = 4;
static int jsr_ext_pcl = 4;
static int jsr_ext_pch = 5;
static int jsr_idx_pcl = 3;
static int jsr_idx_pch = 4;

// ====================================================================
// Forward declarations
// ====================================================================

static InstrType instr_table_6800[];

// ====================================================================
// Helper Methods
// ====================================================================

static int compare_FLAGS(int operand) {
   if (H >= 0) {
      if (H != ((operand >> 5) & 1)) {
         return 1;
      }
   }
   if (I >= 0) {
      if (I != ((operand >> 4) & 1)) {
         return 1;
      }
   }
   if (N >= 0) {
      if (N != ((operand >> 3) & 1)) {
         return 1;
      }
   }
   if (Z >= 0) {
      if (Z != ((operand >> 2) & 1)) {
         return 1;
      }
   }
   if (V >= 0) {
      if (V != ((operand >> 1) & 1)) {
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
   if (operand >= 0) {
      H = (operand >> 5) & 1;
      I = (operand >> 4) & 1;
      N = (operand >> 3) & 1;
      Z = (operand >> 2) & 1;
      V = (operand >> 1) & 1;
      C = (operand >> 0) & 1;
   } else {
      H = -1;
      I = -1;
      N = -1;
      Z = -1;
      V = -1;
      C = -1;
   }
}

static int get_FLAGS() {
   if (H >= 0 && I >= 0 && N >= 0 && Z >= 0 && V >= 0 && C >= 0) {
      return 0xC0 | (H << 5) | (I << 4) | (N << 3) | (Z << 2) | (V << 1) | C;
   } else {
      return -1;
   }
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

static void set_NZV_unknown() {
   N = -1;
   Z = -1;
   V = -1;
}

static void set_NZCV_unknown() {
   N = -1;
   V = -1;
   Z = -1;
   C = -1;
}

static void set_NZ(int value) {
   N = (value >> 7) & 1;
   Z = value == 0;
}

static void set_NZ16(int value) {
   N = (value >> 15) & 1;
   Z = value == 0;
}

static void pop8(int value) {
   if (S >= 0) {
      S = (S + 1) & 0xffff;
      memory_read(value & 0xff, S, MEM_STACK);
   }
}

static void push8(int value) {
   if (S >= 0) {
      memory_write(value & 0xff, S, MEM_STACK);
      S = (S - 1) & 0xffff;
   }
}

static void push16(int value) {
   push8(value);
   push8(value >> 8);
}

static void interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction, int pc_offset) {
   // Parse the bus cycles
   // <opcode> <op1> <write pch> <write pcl> <write p> <read rst> <read rsth>
   int pc     = sample_q[2].data + (sample_q[3].data << 8);
   int x      = sample_q[4].data + (sample_q[5].data << 8);
   int a      = sample_q[6].data;
   int b      = sample_q[7].data;
   int flags  = sample_q[8].data;
   int vector = (sample_q[10].data << 8) + sample_q[11].data;
   // Update the address of the interruted instruction
   instruction->pc = (pc - pc_offset) & 0xffff;
   // Stack the PB/PC/FLags (for memory modelling)
   push16(pc);
   push16(x);
   push8(a);
   push8(b);
   push8(flags);
   // Validate the flags
   check_FLAGS(flags);
   // And make them consistent
   set_FLAGS(flags);
   // Validate the registers
   if ((A >= 0 && A != a) || (B >= 0 && B != b) || (X >= 0 && X != x)) {
      failflag = 1;
   }
   // And make them consistent
   A = a;
   B = b;
   X = x;
   // Setup expected state for the ISR
   I = 1;
   PC = vector;
}

// TODO: this should be somewhere common!

// Being very lazy here using an array!
static char *symbol_table[0x10000];

static void init_symbols() {
   for (int i = 0; i < 0x10000;i++) {
      symbol_table[i] = NULL;
   }
}

// ====================================================================
// Public Methods
// ====================================================================

static void em_6800_init(arguments_t *args) {

   init_symbols();

   instr_table = instr_table_6800;

   // Initialize the SP
   if (args->sp_reg >= 0) {
      S = args->sp_reg & 0xffff;
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


static int em_6800_match_interrupt(sample_t *sample_q, int num_samples) {
   // Check we have enough valid samples
   if (num_samples < 12) {
      return 0;
   }
   // An interupt will write PCH, PCL, PSW in bus cycles 2,3,4
   if (sample_q[0].rnw >= 0) {
      // If we have the RNW pin connected, then just look for these three writes in succession
      // Currently can't detect a WAI or SWI being interrupted
      if (sample_q[0].data == 0x3E || sample_q[0].data == 0x3F) {
         return 0;
      }
      if (sample_q[2].rnw == 0 && sample_q[3].rnw == 0 &&
          sample_q[4].rnw == 0 && sample_q[5].rnw == 0 &&
          sample_q[6].rnw == 0 && sample_q[7].rnw == 0 &&
          sample_q[8].rnw == 0) {
         return 1;
      }
   } else {
      // If not, then we use a heuristic, based on what we expect to see on the data
      // bus in cycles 2, 3 and 8, i.e. PCL, PCH, PSW
      // (we could include X,A,B checks as well).
      if (sample_q[2].data == (PC & 0xff) && sample_q[3].data == ((PC >> 8) & 0xff) ) {
         // Now test unused flag is 1, B is 0
         if ((sample_q[8].data & 0xC0) == 0xC0) {
            // Finally test all other known flags match
            if (!compare_FLAGS(sample_q[8].data & 0x3F)) {
               // Matched PSW = --HIVZVC
               return 1;
            }
         }
      }
   }
   return 0;
}

static int em_6800_count_cycles(sample_t *sample_q, int intr_seen) {
   if (intr_seen) {
      return 12;
   }
   int opcode = sample_q[0].data;
   return instr_table[opcode].cycles;
}

static void em_6800_reset(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   instruction->pc = -1;
   A = -1;
   B = -1;
   X = -1;
   S = -1;
   H = -1;
   I = 1;
   N = -1;
   Z = -1;
   V = -1;
   C = -1;
   PC = (sample_q[num_cycles - 2].data << 8) + sample_q[num_cycles - 1].data;
}

static void em_6800_interrupt(sample_t *sample_q, int num_cycles, instruction_t *instruction) {
   interrupt(sample_q, num_cycles, instruction, 0);
}

static void em_6800_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction) {

   // Unpack the instruction bytes
   int opcode = sample_q[0].data;

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];
   int opcount = instr->len - 1;
   int op1 = (opcount < 1) ? 0 : sample_q[1].data;
   int op2 = (opcount < 2) ? 0 : sample_q[2].data;

   // Memory Modelling: Instruction fetches
   if (PC >= 0) {
      int pc = PC;
      memory_read(opcode, pc++, MEM_FETCH);
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
   if (opcode == 0x3E || opcode == 0x3F) {
      // WAI and SWI
      interrupt(sample_q, num_cycles, instruction, 1); // This handles everything
      return;
   } else if (opcode == 0x8D) {
      // BSR REL
      instruction->pc = (((sample_q[bsr_rel_pch].data << 8) + sample_q[bsr_rel_pcl].data) - 2) & 0xffff;
   } else if (opcode == 0xAD) {
      // JSR IDX
      instruction->pc = (((sample_q[jsr_idx_pch].data << 8) + sample_q[jsr_idx_pcl].data) - 2) & 0xffff;
      // JSR EXT
   } else if (opcode == 0xBD) {
      instruction->pc = (((sample_q[jsr_ext_pch].data << 8) + sample_q[jsr_ext_pcl].data) - 3) & 0xffff;
   } else {
      // current PC value
      instruction->pc = PC;
   }

   // Update PC to start of next instruction
   if (PC >= 0) {
      PC = (PC + instr->len) & 0xffff;
   }

   if (instr->emulate) {

      // For instructions that read or write memory, we need to work out the effective address
      // Note: not needed for stack operations, as S is used directly
      int ea = -1;
      switch (instr->mode) {
      case DIR8:
      case DIR16:
         ea = op1;
         break;
      case EXT8:
      case EXT16:
         ea = (op1 << 8) | op2;
         break;
      case IDX8:
      case IDX16:
         if (X >= 0) {
            ea = (op1 + X) & 0xFFFF;
         }
         break;
      case REL:
         if (PC >= 0) {
            ea = (PC + (int8_t)op1) & 0xFFFF;
         }
      default:
         // covers INH, ACC, IMM8, IMM16
         break;
      }

      int word = instr->mode == DIR16 || instr->mode == EXT16 || instr->mode == IDX16;

      int operand;
      if (instr->optype == RMWOP) {
         // e.g. <opcode> <op1> <op2> <read old> <write old> <write new>
         //      <opcode> <op1>       <read old> <write old> <write new>
         // Want to pick off the read
         operand = sample_q[num_cycles - 3].data;
      } else if (instr->mode == IMM8) {
         // Immediate addressing mode: the operand is the 2nd byte of the instruction
         operand = op1;
      } else if (instr->mode == IMM16) {
         // Immediate addressing mode: the operand is the 2nd byte of the instruction
         operand = (op1 << 8) + op2;
      } else if (instr->mode == REL) {
         // Relative addressing mode: the operand is the 2nd byte of the instruction
         operand = op1;
      } else if (word) {
         // 16 bit data (LDS/LDX/STS/STX/CPX), default top last two bus cycles as operand
         operand = (sample_q[num_cycles - 2].data << 8) + sample_q[num_cycles - 1].data;
         word = 1;
      } else {
         // 8 bit data, default to using the last bus cycle as the operand
         operand = sample_q[num_cycles - 1].data;
      }

      // Operand 2 is the value written back in a store or read-modify-write
      // See RMW comment above for bus cycles
      operand_t operand2 = operand;
      if (instr->optype == RMWOP || instr->optype == WRITEOP) {
         if (word) {
            operand2 = (sample_q[num_cycles - 2].data << 8) + sample_q[num_cycles - 1].data;
         } else {
            operand2 = sample_q[num_cycles - 1].data;
         }
      }

      // Model memory reads
      if (ea >= 0 && (instr->optype == READOP || instr->optype == RMWOP)) {
         if (word) {
            memory_read((operand >> 8) & 0xff,                ea, MEM_DATA);
            memory_read( operand       & 0xff, (ea + 1) & 0xffff, MEM_DATA);
         } else {
            memory_read(operand, ea, MEM_DATA);
         }
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
            if (word) {
               memory_write((operand2 >> 8) & 0xff,                ea, MEM_DATA);
               memory_write( operand2       & 0xff, (ea + 1) & 0xffff, MEM_DATA);
            } else {
               memory_write(operand2,  ea, MEM_DATA);
            }
         }
      }
   }
}

static int em_6800_disassemble(char *buffer, instruction_t *instruction) {

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
   case INH:
   case ACC:
      numchars = sprintf(buffer, fmt, mnemonic);
      break;
   case IMM8:
   case DIR8:
   case DIR16:
   case IDX8:
   case IDX16:
      numchars = sprintf(buffer, fmt, mnemonic, op1);
      break;
   case IMM16:
   case EXT8:
   case EXT16:
      numchars = sprintf(buffer, fmt, mnemonic, op1, op2);
      break;
   case REL:
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
   default:
      numchars = 0;
   }
   return numchars;
}

static int em_6800_get_PC() {
   return PC;
}

static int em_6800_get_PB() {
   return 0;
}

static int em_6800_read_memory(int address) {
   return memory_read_raw(address);
}

static char *em_6800_get_state(char *buffer) {
   strcpy(buffer, default_state);
   if (A >= 0) {
      write_hex2(buffer + OFFSET_A, A);
   }
   if (B >= 0) {
      write_hex2(buffer + OFFSET_B, B);
   }
   if (X >= 0) {
      write_hex4(buffer + OFFSET_X, X);
   }
   if (S >= 0) {
      write_hex4(buffer + OFFSET_S, S);
   }
   if (H >= 0) {
      buffer[OFFSET_H] = '0' + H;
   }
   if (I >= 0) {
      buffer[OFFSET_I] = '0' + I;
   }
   if (N >= 0) {
      buffer[OFFSET_N] = '0' + N;
   }
   if (Z >= 0) {
      buffer[OFFSET_Z] = '0' + Z;
   }
   if (V >= 0) {
      buffer[OFFSET_V] = '0' + V;
   }
   if (C >= 0) {
      buffer[OFFSET_C] = '0' + C;
   }
   return buffer + OFFSET_END;
}

static int em_6800_get_and_clear_fail() {
   int ret = failflag;
   failflag = 0;
   return ret;
}

static void em_6800_symbol_add(char *name, int address) {
   if (address < 0 || address > 0xFFFF) {
      fprintf(stderr, "symbol %s:%04x out of range\r\n", name, address);
      exit(1);
   }
   char *copy = (char *)malloc(strlen(name)+1);
   strcpy(copy, name);
   symbol_table[address] = copy;
}

static char *em_6800_symbol_lookup(int address) {
   if (address >= 0 || address <= 0xFFFF) {
      return symbol_table[address];
   } else {
      return NULL;
   }
}

cpu_emulator_t em_6800 = {
   .init = em_6800_init,
   .match_interrupt = em_6800_match_interrupt,
   .count_cycles = em_6800_count_cycles,
   .reset = em_6800_reset,
   .interrupt = em_6800_interrupt,
   .emulate = em_6800_emulate,
   .disassemble = em_6800_disassemble,
   .get_PC = em_6800_get_PC,
   .get_PB = em_6800_get_PB,
   .read_memory = em_6800_read_memory,
   .get_state = em_6800_get_state,
   .get_and_clear_fail = em_6800_get_and_clear_fail,
   .symbol_add = em_6800_symbol_add,
   .symbol_lookup = em_6800_symbol_lookup
};

// ====================================================================
// Instruction helpers
// ====================================================================

static int add_helper(int val, int operand, int carry) {
   if (val >= 0 && operand >= 0 && carry >= 0) {
      int tmp = val + operand + carry;
      C = (tmp >> 8) & 1;
      V = (((val ^ operand) & 0x80) == 0) && (((val ^ tmp) & 0x80) != 0);
      // Taken from the 6809 decoder
      H =  ((val ^ operand ^ tmp) >> 4) & 1;
      val = tmp & 0xff;
      set_NZ(val);
   } else {
      val = -1;
      set_NZCV_unknown();
   }
   return val;
}

static int and_helper(int val, operand_t operand) {
   if (val >= 0 || operand == 0) {
      val = val & operand;
      set_NZ(val);
   } else {
      set_NZ_unknown();
   }
   V = 0;
   return val;
}

static int asl_helper(int val) {
   if (val >= 0) {
      C = (val >> 7) & 1;
      val = (val << 1) & 0xff;
      set_NZ(val);
      V = C ^ N;
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int asr_helper(int val) {
   if (val >= 0) {
      C = val & 1;
      val = (val & 0x80) | (val >> 1);
      set_NZ(val);
      V = C ^ N;
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int clr_helper() {
   C = 0;
   V = 0;
   N = 0;
   Z = 1;
   return 0;
}

static int com_helper(int val) {
   C = 1;
   V = 0;
   if (val >= 0) {
      val = 0xFF - val;
      set_NZ(val);
   } else {
      set_NZ_unknown();
   }
   return val;
}

static int dec_helper(int val) {
   if (val >= 0) {
      val = (val - 1) & 0xff;
      set_NZ(val);
      V = (val == 0x7F);
   } else {
      set_NZV_unknown();
   }
   return val;
}

static int eor_helper(int val, operand_t operand) {
   if (val >= 0) {
      val = val ^ operand;
      set_NZ(val);
   } else {
      set_NZ_unknown();
   }
   V = 0;
   return val;
}

static int inc_helper(int val) {
   if (val >= 0) {
      val = (val + 1) & 0xff;
      set_NZ(val);
      V = (val == 0x80);
   } else {
      set_NZV_unknown();
   }
   return val;
}

static int lda_helper(int val) {
   set_NZ(val);
   V = 0;
   return val;
}

static int lsr_helper(int val) {
   if (val >= 0) {
      C = val & 1;
      val = val >> 1;
      set_NZ(val);
      V = C ^ N;
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int neg_helper(int val) {
   if (val >= 0) {
      val = (0x00 - val) & 0xff;
      set_NZ(val);
      V = (val == 0x80);
      C = (val == 0x00);
      val = 0xFF - val;
      set_NZ(val);
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int ora_helper(int val, operand_t operand) {
   if (val >= 0) {
      val = val | operand;
      set_NZ(val);
   } else {
      set_NZ_unknown();
   }
   V = 0;
   return val;
}

static int psh_helper(int val, operand_t operand) {
   push8(operand);
   if (val >= 0) {
      if (operand != val) {
         failflag = 1;
      }
   }
   return operand;
}

static int pul_helper(operand_t operand) {
   pop8(operand);
   return operand;
}

static int rol_helper(int val) {
   if (val >= 0) {
      int oldC = C;
      C = (val >> 7) & 1;
      val = ((val << 1) | oldC) & 0xff;
      set_NZ(val);
      V = C ^ N;
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int ror_helper(int val) {
   if (val >= 0) {
      int oldC = C;
      C = val & 1;
      val = (val >> 1) | (oldC << 7);
      set_NZ(val);
      V = C ^ N;
   } else {
      set_NZCV_unknown();
   }
   return val;
}

static int sta_helper(int val, operand_t operand) {
   if (val >= 0) {
      if (operand != val) {
         failflag = 1;
      }
   }
   set_NZ(val);
   V = 0;
   return val;
}

static int sub_helper(int val, int operand, int carry) {
   if (val >= 0 && operand >= 0 && carry >= 0) {
      int tmp = val - operand - carry;
      C = (tmp >> 8) & 1;
      V = (((val ^ operand) & 0x80) != 0) && (((val ^ tmp) & 0x80) != 0);
      val = tmp & 0xff;
      set_NZ(val);
   } else {
      val = -1;
      set_NZCV_unknown();
   }
   return val;
}

static int tst_helper(int val) {
   if (val >= 0) {
      set_NZ(val);
   } else {
      set_NZ_unknown();
   }
   V = 0;
   C = 0;
   return val;
}

// ====================================================================
// Individual Instructions
// ====================================================================

static int op_ABA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = add_helper(A, B, 0);
   return -1;
}

static int op_ADCA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = add_helper(A, operand, C);
   return -1;
}
static int op_ADCB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = add_helper(B, operand, C);
   return -1;
}

static int op_ADDA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = add_helper(A, operand, 0);
   return -1;
}

static int op_ADDB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = add_helper(A, operand, 0);
   return -1;
}

static int op_ANDA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = and_helper(A, operand);
   return -1;
}

static int op_ANDB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = and_helper(B, operand);
   return -1;
}

static int op_ASL(operand_t operand, ea_t ea, sample_t *sample_q) {
   return asl_helper(operand);
}

static int op_ASLA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = asl_helper(A);
   return -1;
}

static int op_ASLB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = asl_helper(B);
   return -1;
}

static int op_ASR(operand_t operand, ea_t ea, sample_t *sample_q) {
   return asr_helper(operand);
}

static int op_ASRA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = asr_helper(A);
   return -1;
}

static int op_ASRB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = asr_helper(B);
   return -1;
}

static int op_BCC(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if !C
   if (C == 0) {
      PC = ea;
   } else if (C < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BCS(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (C == 1) {
      PC = ea;
   } else if (C < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BEQ(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Brach if Z=1
   if (Z == 1) {
      PC = ea;
   } else if (Z < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BGE(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if N = V
   if (N >= 0 && V >= 0) {
      if (N == V) {
         PC = ea;
      }
   } else {
      PC = -1;
   }
   return -1;
}

static int op_BGT(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if !Z and N = V
   // TODO: narrow the scope of this test
   if (Z >= 0 && N >= 0 && V >= 0) {
      if (!Z && (N == V)) {
         PC = ea;
      }
   } else {
      PC = -1;
   }
   return -1;
}

static int op_BHI(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if !C and !Z
   //  C  Z    taken
   // -1 -1 => -1
   // -1  0 => -1
   // -1  1 =>  0
   //  0 -1 => -1
   //  0  0 =>  1
   //  0  1 =>  0
   //  1 -1 =>  0
   //  1  0 =>  0
   //  1  1 =>  0
   if (C == 0 && Z == 0) {
      PC = ea;
   } else if ((C < 0 && Z != 1) || (Z < 0 && C != 1)) {
      PC = -1;
   }
   return -1;
}

static int op_BITA(operand_t operand, ea_t ea, sample_t *sample_q) {
   and_helper(A, operand);
   return -1;
}

static int op_BITB(operand_t operand, ea_t ea, sample_t *sample_q) {
   and_helper(B, operand);
   return -1;
}

static int op_BLE(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if Z OR N != V
   // TODO: narrow the scope of this test
   if (Z >= 0 && N >= 0 && V >= 0) {
      if (Z || (N != V)) {
         PC = ea;
      }
   } else {
      PC = -1;
   }
   return -1;
}

static int op_BLS(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if C or Z
   //  C  Z    taken
   // -1 -1 => -1
   // -1  0 => -1
   // -1  1 =>  1
   //  0 -1 => -1
   //  0  0 =>  0
   //  0  1 =>  1
   //  1 -1 =>  1
   //  1  0 =>  1
   //  1  1 =>  1
   if (C == 1 || Z == 1) {
      PC = ea;
   } else if (C != 0 && Z != 0) {
      PC = -1;
   }
   return -1;
}

static int op_BLT(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if N != V
   if (N >= 0 && V >= 0) {
      if (N != V) {
         PC = ea;
      }
   } else {
      PC = -1;
   }
   return -1;
}

static int op_BMI(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if N=1
   if (N == 1) {
      PC = ea;
   } else if (N < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BNE(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if Z=0
   if (Z == 0) {
      PC = ea;
   } else if (Z < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BPL(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if N=0
   if (N == 0) {
      PC = ea;
   } else if (N < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BRA(operand_t operand, ea_t ea, sample_t *sample_q) {
   PC = ea;
   return -1;
}

static int op_BSR(operand_t operand, ea_t ea, sample_t *sample_q) {
   int idx = bsr_rel_pcl;
   push8(sample_q[idx].data);
   push8(sample_q[idx + 1].data);
   PC = ea;
   return -1;
}

static int op_BVC(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if V=0
   if (V == 0) {
      PC = ea;
   } else if (V < 0) {
      PC = -1;
   }
   return -1;
}

static int op_BVS(operand_t operand, ea_t ea, sample_t *sample_q) {
   // Branch if V=1
   if (V == 1) {
      PC = ea;
   } else if (V < 0) {
      PC = -1;
   }
   return -1;
}

static int op_CBA(operand_t operand, ea_t ea, sample_t *sample_q) {
   sub_helper(A, B, 0);
   return -1;
}

static int op_CLC(operand_t operand, ea_t ea, sample_t *sample_q) {
   C = 0;
   return -1;
}

static int op_CLI(operand_t operand, ea_t ea, sample_t *sample_q) {
   I = 0;
   return -1;
}

static int op_CLR(operand_t operand, ea_t ea, sample_t *sample_q) {
   return clr_helper();
}

static int op_CLRA(operand_t operand, ea_t ea, sample_t *sample_q) {
   return A = clr_helper();
}

static int op_CLRB(operand_t operand, ea_t ea, sample_t *sample_q) {
   return B = clr_helper();
}

static int op_CLV(operand_t operand, ea_t ea, sample_t *sample_q) {
   V = 0;
   return -1;
}

static int op_CMPA(operand_t operand, ea_t ea, sample_t *sample_q) {
   sub_helper(A, operand, 0);
   return -1;
}

static int op_CMPB(operand_t operand, ea_t ea, sample_t *sample_q) {
   sub_helper(B, operand, 0);
   return -1;
}

static int op_COM(operand_t operand, ea_t ea, sample_t *sample_q) {
   return com_helper(operand);
}

static int op_COMA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = com_helper(A);
   return -1;
}

static int op_COMB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = com_helper(B);
   return -1;
}

static int op_CPX(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (X >= 0) {
      int xl = X & 0xff;
      int xh = (X >> 8) & 0xff;
      int opl = operand & 0xff;
      int oph = (operand >> 8) & 0xff;
      int resl = xl - opl;
      int resh = xh - oph;
      Z = !resl && !resh;
      N = (resh & 128) > 0;
      V = (((xh ^ oph) & 0x80) != 0) && (((xh ^ resh) & 0x80) != 0);
   } else {
      set_NZC_unknown();
   }
   return -1;
}

// Taken from the 6809 decoder
static int op_DAA(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0 && H >= 0 && C >= 0) {
      int correction = 0x00;
      if (H == 1 || (A & 0x0f) > 0x09) {
         correction |= 0x06;
      }
      if (C == 1 || (A & 0xf0) > 0x90 || ((A & 0xf0) > 0x80 && (A & 0x0f) > 0x09)) {
         correction |= 0x60;
      }
      int tmp = A + correction;
      // TODO: On the 6809 C is apparently only ever set by DAA, never cleared
      C = (tmp >> 8) & 1;
      // V is is calculated as follows on both the 6809 and the 6309
      V = ((tmp >> 7) & 1) ^ C;
      tmp &= 0xff;
      set_NZ(tmp);
      A = tmp;
   } else {
      A = -1;
      set_NZC_unknown();
   }
   return -1;
}

static int op_DEC(operand_t operand, ea_t ea, sample_t *sample_q) {
   return dec_helper(operand);
}

static int op_DECA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = dec_helper(A);
   return -1;
}

static int op_DECB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = dec_helper(B);
   return -1;
}

static int op_DES(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (S >= 0) {
      S = (S - 1) & 0xFFFF;
   }
   return -1;
}

static int op_DEX(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (X >= 0) {
      X = (X - 1) & 0xFFFF;
      Z = (X == 0);
   }
   return -1;
}

static int op_EORA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = eor_helper(A, operand);
   return -1;
}

static int op_EORB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = eor_helper(B, operand);
   return -1;
}

static int op_INC(operand_t operand, ea_t ea, sample_t *sample_q) {
   return inc_helper(operand);
}

static int op_INCA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = inc_helper(A);
   return -1;
}

static int op_INCB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = inc_helper(B);
   return -1;
}

static int op_INS(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (S >= 0) {
      S = (S + 1) & 0xFFFF;
   }
   return -1;
}

static int op_INX(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (X >= 0) {
      X = (X + 1) & 0xFFFF;
      Z = (X == 0);
   }
   return -1;
}

static int op_JMP(operand_t operand, ea_t ea, sample_t *sample_q) {
   PC = ea;
   return -1;
}

static int op_JSR(operand_t operand, ea_t ea, sample_t *sample_q) {
   PC = ea;
   int idx = sample_q[0].data == 0xAD ? jsr_idx_pcl : jsr_ext_pcl;
   push8(sample_q[idx].data);
   push8(sample_q[idx + 1].data);
   return -1;
}

static int op_LDAA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = lda_helper(operand);
   return -1;
}

static int op_LDAB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = lda_helper(operand);
   return -1;
}

static int op_LDS(operand_t operand, ea_t ea, sample_t *sample_q) {
   S = operand;
   set_NZ16(S);
   V = 0;
   return -1;
}

static int op_LDX(operand_t operand, ea_t ea, sample_t *sample_q) {
   X = operand;
   set_NZ16(X);
   V = 0;
   return -1;
}

static int op_LSR(operand_t operand, ea_t ea, sample_t *sample_q) {
   return lsr_helper(operand);
}

static int op_LSRA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = lsr_helper(A);
   return -1;
}

static int op_LSRB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = lsr_helper(B);
   return -1;
}

static int op_NEG(operand_t operand, ea_t ea, sample_t *sample_q) {
   return neg_helper(operand);
}

static int op_NEGA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = neg_helper(A);
   return -1;
}

static int op_NEGB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = neg_helper(B);
   return -1;
}

static int op_ORAA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = ora_helper(A, operand);
   return -1;
}

static int op_ORAB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = ora_helper(B, operand);
   return -1;
}

static int op_PSHA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = psh_helper(A, operand);
   return -1;
}

static int op_PSHB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = psh_helper(B, operand);
   return -1;
}

static int op_PULA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = pul_helper(operand);
   return -1;
}

static int op_PULB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = pul_helper(operand);
   return -1;
}

static int op_ROL(operand_t operand, ea_t ea, sample_t *sample_q) {
   return rol_helper(operand);
}

static int op_ROLA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = rol_helper(A);
   return -1;
}

static int op_ROLB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = rol_helper(B);
   return -1;
}

static int op_ROR(operand_t operand, ea_t ea, sample_t *sample_q) {
   return ror_helper(operand);
}

static int op_RORA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = ror_helper(A);
   return -1;
}

static int op_RORB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = ror_helper(B);
   return -1;
}

static int op_RTI(operand_t operand, ea_t ea, sample_t *sample_q) {
   // 0 = opcode
   // 1 = dead
   // 2 = dead
   // 3 = flags
   // 4 = B
   // 5 = A
   // 6 = XH
   // 7 = XL
   // 8 = PCH
   // 9 = PCL
   for (int i = 3; i <= 9; i++) {
      pop8(sample_q[i].data);
   }
   set_FLAGS(sample_q[3].data);
   B = sample_q[4].data;
   A = sample_q[5].data;
   X = (sample_q[6].data << 8) + sample_q[7].data;
   PC = (sample_q[8].data << 8) + sample_q[9].data;
   return -1;
}

static int op_RTS(operand_t operand, ea_t ea, sample_t *sample_q) {
   // 0 = opcode
   // 1 = dead
   // 2 = dead
   // 3 = PCH
   // 4 = PCL
   for (int i = 3; i <= 4; i++) {
      pop8(sample_q[i].data);
   }
   PC = (sample_q[3].data << 8) + sample_q[4].data;
   return -1;
}

static int op_SBA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = sub_helper(A, B, 0);
   return -1;
}

static int op_SBCA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = sub_helper(A, operand, C);
   return -1;
}

static int op_SBCB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = sub_helper(B, operand, C);
   return -1;
}

static int op_SEC(operand_t operand, ea_t ea, sample_t *sample_q) {
   C = 1;
   return -1;
}

static int op_SEI(operand_t operand, ea_t ea, sample_t *sample_q) {
   I = 1;
   return -1;
}

static int op_SEV(operand_t operand, ea_t ea, sample_t *sample_q) {
   V = 1;
   return -1;
}

static int op_STAA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = sta_helper(A, operand);
   return operand;
}

static int op_STAB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = sta_helper(B, operand);
   return operand;
}

static int op_STS(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (S >= 0) {
      if (operand != S) {
         failflag = 1;
      }
   }
   S = operand;
   set_NZ16(S);
   V = 0;
   return operand;
}

static int op_STX(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (X >= 0) {
      if (operand != X) {
         failflag = 1;
      }
   }
   X = operand;
   set_NZ16(X);
   V = 0;
   return operand;
}

static int op_SUBA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = sub_helper(A, operand, 0);
   return -1;
}

static int op_SUBB(operand_t operand, ea_t ea, sample_t *sample_q) {
   B = sub_helper(B, operand, 0);
   return -1;
}

static int op_TAB(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (A >= 0) {
      B = A;
      set_NZ(B);
   } else {
      B = -1;
      set_NZ_unknown();
   }
   V = 0;
   return -1;
}

static int op_TAP(operand_t operand, ea_t ea, sample_t *sample_q) {
   set_FLAGS(A);
   return -1;
}

static int op_TBA(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (B >= 0) {
      A = B;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
   V = 0;
   return -1;
}

static int op_TPA(operand_t operand, ea_t ea, sample_t *sample_q) {
   A = get_FLAGS();
   return -1;
}

static int op_TST(operand_t operand, ea_t ea, sample_t *sample_q) {
   tst_helper(operand);
   return -1;
}

static int op_TSTA(operand_t operand, ea_t ea, sample_t *sample_q) {
   tst_helper(A);
   return -1;
}

static int op_TSTB(operand_t operand, ea_t ea, sample_t *sample_q) {
   tst_helper(B);
   return -1;
}

static int op_TSX(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (S >= 0) {
      X = (S + 1) & 0xffff;
   } else {
      X = -1;
   }
   return -1;
}

static int op_TXS(operand_t operand, ea_t ea, sample_t *sample_q) {
   if (X >= 0) {
      S = (X - 1) & 0xffff;
   } else {
      S = -1;
   }
   return -1;
}

static int op_WAI(operand_t operand, ea_t ea, sample_t *sample_q) {
   // TODO
   return -1;
}

// ====================================================================
// Opcode Tables
// ====================================================================

static InstrType instr_table_6800[] = {
   /* 00 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 01 */   { "NOP  ", 0,   INH,  2,     OTHER,       0},
   /* 02 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 03 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 04 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 05 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 06 */   { "TAP  ", 0,   INH,  2,     OTHER,  op_TAP},
   /* 07 */   { "TPA  ", 0,   INH,  2,     OTHER,  op_TPA},
   /* 08 */   { "INX  ", 0,   INH,  4,     OTHER,  op_INX},
   /* 09 */   { "DEX  ", 0,   INH,  4,     OTHER,  op_DEX},
   /* 0A */   { "CLV  ", 0,   INH,  2,     OTHER,  op_CLV},
   /* 0B */   { "SEV  ", 0,   INH,  2,     OTHER,  op_SEV},
   /* 0C */   { "CLC  ", 0,   INH,  2,     OTHER,  op_CLC},
   /* 0D */   { "SEC  ", 0,   INH,  2,     OTHER,  op_SEC},
   /* 0E */   { "CLI  ", 0,   INH,  2,     OTHER,  op_CLI},
   /* 0F */   { "SEI  ", 0,   INH,  2,     OTHER,  op_SEI},

   /* 10 */   { "SBA  ", 0,   INH,  2,     OTHER,  op_SBA},
   /* 11 */   { "CBA  ", 0,   INH,  2,     OTHER,  op_CBA},
   /* 12 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 13 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 14 */   { "NBA  ", 1,   INH,  2,     OTHER,       0},
   /* 15 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 16 */   { "TAB  ", 0,   INH,  2,     OTHER,  op_TAB},
   /* 17 */   { "TBA  ", 0,   INH,  2,     OTHER,  op_TBA},
   /* 18 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 19 */   { "DAA  ", 0,   INH,  2,     OTHER,  op_DAA},
   /* 1A */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 1B */   { "ABA  ", 0,   INH,  2,     OTHER,  op_ABA},
   /* 1C */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 1D */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 1E */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 1F */   { "???  ", 1,   INH,  2,     OTHER,       0},

   /* 20 */   { "BRA  ", 0,   REL,  4,  BRANCHOP,  op_BRA},
   /* 21 */   { "???  ", 1,   REL,  4,  BRANCHOP,       0},
   /* 22 */   { "BHI  ", 0,   REL,  4,  BRANCHOP,  op_BHI},
   /* 23 */   { "BLS  ", 0,   REL,  4,  BRANCHOP,  op_BLS},
   /* 24 */   { "BCC  ", 0,   REL,  4,  BRANCHOP,  op_BCC},
   /* 25 */   { "BCS  ", 0,   REL,  4,  BRANCHOP,  op_BCS},
   /* 26 */   { "BNE  ", 0,   REL,  4,  BRANCHOP,  op_BNE},
   /* 27 */   { "BEQ  ", 0,   REL,  4,  BRANCHOP,  op_BEQ},
   /* 28 */   { "BVC  ", 0,   REL,  4,  BRANCHOP,  op_BVC},
   /* 29 */   { "BVS  ", 0,   REL,  4,  BRANCHOP,  op_BVS},
   /* 2A */   { "BPL  ", 0,   REL,  4,  BRANCHOP,  op_BPL},
   /* 2B */   { "BMI  ", 0,   REL,  4,  BRANCHOP,  op_BMI},
   /* 2C */   { "BGE  ", 0,   REL,  4,  BRANCHOP,  op_BGE},
   /* 2D */   { "BLT  ", 0,   REL,  4,  BRANCHOP,  op_BLT},
   /* 2E */   { "BGT  ", 0,   REL,  4,  BRANCHOP,  op_BGT},
   /* 2F */   { "BLE  ", 0,   REL,  4,  BRANCHOP,  op_BLE},

   /* 30 */   { "TSX  ", 0,   INH,  4,     OTHER,  op_TSX},
   /* 31 */   { "INS  ", 0,   INH,  4,     OTHER,  op_INS},
   /* 32 */   { "PUL A", 0,   ACC,  4,     OTHER, op_PULA},
   /* 33 */   { "PUL B", 0,   ACC,  4,     OTHER, op_PULB},
   /* 34 */   { "DES  ", 0,   INH,  4,     OTHER,  op_DES},
   /* 35 */   { "TXS  ", 0,   INH,  4,     OTHER,  op_TXS},
   /* 36 */   { "PSH A", 0,   ACC,  4,     OTHER, op_PSHA},
   /* 37 */   { "PSH B", 0,   ACC,  4,     OTHER, op_PSHB},
   /* 38 */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 39 */   { "RTS  ", 0,   INH,  5,     OTHER,  op_RTS},
   /* 3A */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 3B */   { "RTI  ", 0,   INH, 10,     OTHER,  op_RTI},
   /* 3C */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 3D */   { "???  ", 1,   INH,  2,     OTHER,       0},
   /* 3E */   { "WAI  ", 0,   INH,  9,     OTHER,  op_WAI},
   /* 3F */   { "SWI  ", 0,   INH, 12,     OTHER,       0},

   /* 40 */   { "NEG A", 0,   ACC,  2,     OTHER, op_NEGA},
   /* 41 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 42 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 43 */   { "COM A", 0,   ACC,  2,     OTHER, op_COMA},
   /* 44 */   { "LSR A", 0,   ACC,  2,     OTHER, op_LSRA},
   /* 45 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 46 */   { "ROR A", 0,   ACC,  2,     OTHER, op_RORA},
   /* 47 */   { "ASR A", 0,   ACC,  2,     OTHER, op_ASRA},
   /* 48 */   { "ASL A", 0,   ACC,  2,     OTHER, op_ASLA},
   /* 49 */   { "ROL A", 0,   ACC,  2,     OTHER, op_ROLA},
   /* 4A */   { "DEC A", 0,   ACC,  2,     OTHER, op_DECA},
   /* 4B */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 4C */   { "INC A", 0,   ACC,  2,     OTHER, op_INCA},
   /* 4D */   { "TST A", 0,   ACC,  2,     OTHER, op_TSTA},
   /* 4E */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 4F */   { "CLR A", 0,   ACC,  2,     OTHER, op_CLRA},

   /* 50 */   { "NEG B", 0,   ACC,  2,     OTHER, op_NEGB},
   /* 51 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 52 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 53 */   { "COM B", 0,   ACC,  2,     OTHER, op_COMB},
   /* 54 */   { "LSR B", 0,   ACC,  2,     OTHER, op_LSRB},
   /* 55 */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 56 */   { "ROR B", 0,   ACC,  2,     OTHER, op_RORB},
   /* 57 */   { "ASR B", 0,   ACC,  2,     OTHER, op_ASRB},
   /* 58 */   { "ASL B", 0,   ACC,  2,     OTHER, op_ASLB},
   /* 59 */   { "ROL B", 0,   ACC,  2,     OTHER, op_ROLB},
   /* 5A */   { "DEC B", 0,   ACC,  2,     OTHER, op_DECB},
   /* 5B */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 5C */   { "INC B", 0,   ACC,  2,     OTHER, op_INCB},
   /* 5D */   { "TST B", 0,   ACC,  2,     OTHER, op_TSTB},
   /* 5E */   { "???  ", 1,   ACC,  2,     OTHER,       0},
   /* 5F */   { "CLR B", 0,   ACC,  2,     OTHER, op_CLRB},

   /* 60 */   { "NEG  ", 0,  IDX8,  7,     RMWOP, op_NEG},
   /* 61 */   { "???  ", 1,  IDX8,  7,     OTHER,       0},
   /* 62 */   { "???  ", 1,  IDX8,  7,     OTHER,       0},
   /* 63 */   { "COM  ", 0,  IDX8,  7,     RMWOP,  op_COM},
   /* 64 */   { "LSR  ", 0,  IDX8,  7,     RMWOP,  op_LSR},
   /* 65 */   { "???  ", 1,  IDX8,  7,     OTHER,       0},
   /* 66 */   { "ROR  ", 0,  IDX8,  7,     RMWOP,  op_ROR},
   /* 67 */   { "ASR  ", 0,  IDX8,  7,     RMWOP,  op_ASR},
   /* 68 */   { "ASL  ", 0,  IDX8,  7,     RMWOP,  op_ASL},
   /* 69 */   { "ROL  ", 0,  IDX8,  7,     RMWOP,  op_ROL},
   /* 6A */   { "DEC  ", 0,  IDX8,  7,     RMWOP,  op_DEC},
   /* 6B */   { "???  ", 1,  IDX8,  7,     OTHER,       0},
   /* 6C */   { "INC  ", 0,  IDX8,  7,     RMWOP,  op_INC},
   /* 6D */   { "TST  ", 0,  IDX8,  7,    READOP,  op_TST},
   /* 6E */   { "JMP  ", 0,  IDX8,  4,  JSRJMPOP,  op_JMP},
   /* 6F */   { "CLR  ", 0,  IDX8,  7,   WRITEOP,  op_CLR},

   /* 70 */   { "NEG  ", 0,  EXT8,  6,     RMWOP,  op_NEG},
   /* 71 */   { "???  ", 1,  EXT8,  6,     OTHER,       0},
   /* 72 */   { "???  ", 1,  EXT8,  6,     OTHER,       0},
   /* 73 */   { "COM  ", 0,  EXT8,  6,     RMWOP,  op_COM},
   /* 74 */   { "LSR  ", 0,  EXT8,  6,     RMWOP,  op_LSR},
   /* 75 */   { "???  ", 1,  EXT8,  6,     OTHER,       0},
   /* 76 */   { "ROR  ", 0,  EXT8,  6,     RMWOP,  op_ROR},
   /* 77 */   { "ASR  ", 0,  EXT8,  6,     RMWOP,  op_ASR},
   /* 78 */   { "ASL  ", 0,  EXT8,  6,     RMWOP,  op_ASL},
   /* 79 */   { "ROL  ", 0,  EXT8,  6,     RMWOP,  op_ROL},
   /* 7A */   { "DEC  ", 0,  EXT8,  6,     RMWOP,  op_DEC},
   /* 7B */   { "???  ", 1,  EXT8,  6,     OTHER,       0},
   /* 7C */   { "INC  ", 0,  EXT8,  6,     RMWOP,  op_INC},
   /* 7D */   { "TST  ", 0,  EXT8,  6,    READOP,  op_TST},
   /* 7E */   { "JMP  ", 0,  EXT8,  3,  JSRJMPOP,  op_JMP},
   /* 7F */   { "CLR  ", 0,  EXT8,  6,   WRITEOP,  op_CLR},

   /* 80 */   { "SUB A", 0,  IMM8,  2,     OTHER, op_SUBA},
   /* 81 */   { "CMP A", 0,  IMM8,  2,     OTHER, op_CMPA},
   /* 82 */   { "SBC A", 0,  IMM8,  2,     OTHER, op_SBCA},
   /* 83 */   { "SBC?A", 1,  IMM8,  2,     OTHER,       0},
   /* 84 */   { "AND A", 0,  IMM8,  2,     OTHER, op_ANDA},
   /* 85 */   { "BIT A", 0,  IMM8,  2,     OTHER, op_BITA},
   /* 86 */   { "LDA A", 0,  IMM8,  2,     OTHER, op_LDAA},
   /* 87 */   { "STA A", 1,  IMM8,  2,     OTHER,       0},
   /* 88 */   { "EOR A", 0,  IMM8,  2,     OTHER, op_EORA},
   /* 89 */   { "ADC A", 0,  IMM8,  2,     OTHER, op_ADCA},
   /* 8A */   { "ORA A", 0,  IMM8,  2,     OTHER, op_ORAA},
   /* 8B */   { "ADD A", 0,  IMM8,  2,     OTHER, op_ADDA},
   /* 8C */   { "CPX  ", 0, IMM16,  3,     OTHER,  op_CPX},
   /* 8D */   { "BSR  ", 0,   REL,  8,  BRANCHOP,  op_BSR},
   /* 8E */   { "LDS  ", 0, IMM16,  3,     OTHER,  op_LDS},
   /* 8F */   { "STS  ", 1, IMM16,  4,     OTHER,       0},

   /* 90 */   { "SUB A", 0,  DIR8,  3,    READOP, op_SUBA},
   /* 91 */   { "CMP A", 0,  DIR8,  3,    READOP, op_CMPA},
   /* 92 */   { "SBC A", 0,  DIR8,  3,    READOP, op_SBCA},
   /* 93 */   { "SBC?A", 1,  DIR8,  3,    READOP,       0},
   /* 94 */   { "AND A", 0,  DIR8,  3,    READOP, op_ANDA},
   /* 95 */   { "BIT A", 0,  DIR8,  3,    READOP, op_BITA},
   /* 96 */   { "LDA A", 0,  DIR8,  3,    READOP, op_LDAA},
   /* 97 */   { "STA A", 0,  DIR8,  4,   WRITEOP, op_STAA},
   /* 98 */   { "EOR A", 0,  DIR8,  3,    READOP, op_EORA},
   /* 99 */   { "ADC A", 0,  DIR8,  3,    READOP, op_ADCA},
   /* 9A */   { "ORA A", 0,  DIR8,  3,    READOP, op_ORAA},
   /* 9B */   { "ADD A", 0,  DIR8,  3,    READOP, op_ADDA},
   /* 9C */   { "CPX  ", 0, DIR16,  4,    READOP,  op_CPX},
   /* 9D */   { "HCF  ", 1,   INH,  3,     OTHER,       0},
   /* 9E */   { "LDS  ", 0, DIR16,  4,    READOP,  op_LDS},
   /* 9F */   { "STS  ", 0, DIR16,  5,   WRITEOP,  op_STS},

   /* A0 */   { "SUB A", 0,  IDX8,  5,    READOP, op_SUBA},
   /* A1 */   { "CMP A", 0,  IDX8,  5,    READOP, op_CMPA},
   /* A2 */   { "SBC A", 0,  IDX8,  5,    READOP, op_SBCA},
   /* A3 */   { "SBC?A", 1,  IDX8,  5,    READOP,       0},
   /* A4 */   { "AND A", 0,  IDX8,  5,    READOP, op_ANDA},
   /* A5 */   { "BIT A", 0,  IDX8,  5,    READOP, op_BITA},
   /* A6 */   { "LDA A", 0,  IDX8,  5,    READOP, op_LDAA},
   /* A7 */   { "STA A", 0,  IDX8,  6,   WRITEOP, op_STAA},
   /* A8 */   { "EOR A", 0,  IDX8,  5,    READOP, op_EORA},
   /* A9 */   { "ADC A", 0,  IDX8,  5,    READOP, op_ADCA},
   /* AA */   { "ORA A", 0,  IDX8,  5,    READOP, op_ORAA},
   /* AB */   { "ADD A", 0,  IDX8,  5,    READOP, op_ADDA},
   /* AC */   { "CPX  ", 0, IDX16,  6,    READOP,  op_CPX},
   /* AD */   { "JSR  ", 0,  IDX8,  8,  JSRJMPOP,  op_JSR},
   /* AE */   { "LDS  ", 0, IDX16,  6,    READOP,  op_LDS},
   /* AF */   { "STS  ", 0, IDX16,  7,   WRITEOP,  op_STS},

   /* B0 */   { "SUB A", 0,  EXT8,  4,    READOP, op_SUBA},
   /* B1 */   { "CMP A", 0,  EXT8,  4,    READOP, op_CMPA},
   /* B2 */   { "SBC A", 0,  EXT8,  4,    READOP, op_SBCA},
   /* B3 */   { "SBC?A", 1,  EXT8,  4,    READOP,       0},
   /* B4 */   { "AND A", 0,  EXT8,  4,    READOP, op_ANDA},
   /* B5 */   { "BIT A", 0,  EXT8,  4,    READOP, op_BITA},
   /* B6 */   { "LDA A", 0,  EXT8,  4,    READOP, op_LDAA},
   /* B7 */   { "STA A", 0,  EXT8,  5,   WRITEOP, op_STAA},
   /* B8 */   { "EOR A", 0,  EXT8,  4,    READOP, op_EORA},
   /* B9 */   { "ADC A", 0,  EXT8,  4,    READOP, op_ADCA},
   /* BA */   { "ORA A", 0,  EXT8,  4,    READOP, op_ORAA},
   /* BB */   { "ADD A", 0,  EXT8,  4,    READOP, op_ADDA},
   /* BC */   { "CPX  ", 0, EXT16,  5,    READOP,  op_CPX},
   /* BD */   { "JSR  ", 0,  EXT8,  9,  JSRJMPOP,  op_JSR},
   /* BE */   { "LDS  ", 0, EXT16,  5,    READOP,  op_LDS},
   /* BF */   { "STS  ", 0, EXT16,  6,   WRITEOP,  op_STS},

   /* C0 */   { "SUB B", 0,  IMM8,  2,     OTHER, op_SUBB},
   /* C1 */   { "CMP B", 0,  IMM8,  2,     OTHER, op_CMPB},
   /* C2 */   { "SBC B", 0,  IMM8,  2,     OTHER, op_SBCB},
   /* C3 */   { "SBC?B", 1,  IMM8,  2,     OTHER,       0},
   /* C4 */   { "AND B", 0,  IMM8,  2,     OTHER, op_ANDB},
   /* C5 */   { "BIT B", 0,  IMM8,  2,     OTHER, op_BITB},
   /* C6 */   { "LDA B", 0,  IMM8,  2,     OTHER, op_LDAB},
   /* C7 */   { "STA B", 1,  IMM8,  2,     OTHER,       0},
   /* C8 */   { "EOR B", 0,  IMM8,  2,     OTHER, op_EORB},
   /* C9 */   { "ADC B", 0,  IMM8,  2,     OTHER, op_ADCB},
   /* CA */   { "ORA B", 0,  IMM8,  2,     OTHER, op_ORAB},
   /* CB */   { "ADD B", 0,  IMM8,  2,     OTHER, op_ADDB},
   /* CC */   { "???  ", 1, IMM16,  3,     OTHER,       0},
   /* CD */   { "HCF  ", 1,   INH,  3,     OTHER,       0},
   /* CE */   { "LDX  ", 0, IMM16,  3,     OTHER,  op_LDX},
   /* CF */   { "STX  ", 1, IMM16,  4,     OTHER,       0},

   /* D0 */   { "SUB B", 0,  DIR8,  3,    READOP, op_SUBB},
   /* D1 */   { "CMP B", 0,  DIR8,  3,    READOP, op_CMPB},
   /* D2 */   { "SBC B", 0,  DIR8,  3,    READOP, op_SBCB},
   /* D3 */   { "SBC?B", 1,  DIR8,  3,    READOP,       0},
   /* D4 */   { "AND B", 0,  DIR8,  3,    READOP, op_ANDB},
   /* D5 */   { "BIT B", 0,  DIR8,  3,    READOP, op_BITB},
   /* D6 */   { "LDA B", 0,  DIR8,  3,    READOP, op_LDAB},
   /* D7 */   { "STA B", 0,  DIR8,  4,   WRITEOP, op_STAB},
   /* D8 */   { "EOR B", 0,  DIR8,  3,    READOP, op_EORB},
   /* D9 */   { "ADC B", 0,  DIR8,  3,    READOP, op_ADCB},
   /* DA */   { "ORA B", 0,  DIR8,  3,    READOP, op_ORAB},
   /* DB */   { "ADD B", 0,  DIR8,  3,    READOP, op_ADDB},
   /* DC */   { "???  ", 1, DIR16,  4,     OTHER,       0},
   /* DD */   { "HCF  ", 1,   INH,  4,     OTHER,       0},
   /* DE */   { "LDX  ", 0, DIR16,  4,    READOP,  op_LDX},
   /* DF */   { "STX  ", 0, DIR16,  5,   WRITEOP,  op_STX},

   /* E0 */   { "SUB B", 0,  IDX8,  5,    READOP, op_SUBB},
   /* E1 */   { "CMP B", 0,  IDX8,  5,    READOP, op_CMPB},
   /* E2 */   { "SBC B", 0,  IDX8,  5,    READOP, op_SBCB},
   /* E3 */   { "SBC?B", 1,  IDX8,  5,    READOP,       0},
   /* E4 */   { "AND B", 0,  IDX8,  5,    READOP, op_ANDB},
   /* E5 */   { "BIT B", 0,  IDX8,  5,    READOP, op_BITB},
   /* E6 */   { "LDA B", 0,  IDX8,  5,    READOP, op_LDAB},
   /* E7 */   { "STA B", 0,  IDX8,  6,   WRITEOP, op_STAB},
   /* E8 */   { "EOR B", 0,  IDX8,  5,    READOP, op_EORB},
   /* E9 */   { "ADC B", 0,  IDX8,  5,    READOP, op_ADCB},
   /* EA */   { "ORA B", 0,  IDX8,  5,    READOP, op_ORAB},
   /* EB */   { "ADD B", 0,  IDX8,  5,    READOP, op_ADDB},
   /* EC */   { "???  ", 1, IDX16,  6,     OTHER,       0},
   /* ED */   { "HCF  ", 1,   INH,  6,     OTHER,       0},
   /* EE */   { "LDX  ", 0, IDX16,  6,    READOP,  op_LDX},
   /* EF */   { "STX  ", 0, IDX16,  7,   WRITEOP,  op_STX},

   /* F0 */   { "SUB B", 0,  EXT8,  4,    READOP, op_SUBB},
   /* F1 */   { "CMP B", 0,  EXT8,  4,    READOP, op_CMPB},
   /* F2 */   { "SBC B", 0,  EXT8,  4,    READOP, op_SBCB},
   /* F3 */   { "SBC?B", 1,  EXT8,  4,    READOP,       0},
   /* F4 */   { "AND B", 0,  EXT8,  4,    READOP, op_ANDB},
   /* F5 */   { "BIT B", 0,  EXT8,  4,    READOP, op_BITB},
   /* F6 */   { "LDA B", 0,  EXT8,  4,    READOP, op_LDAB},
   /* F7 */   { "STA B", 0,  EXT8,  5,   WRITEOP, op_STAB},
   /* F8 */   { "EOR B", 0,  EXT8,  4,    READOP, op_EORB},
   /* F9 */   { "ADC B", 0,  EXT8,  4,    READOP, op_ADCB},
   /* FA */   { "ORA B", 0,  EXT8,  4,    READOP, op_ORAB},
   /* FB */   { "ADD B", 0,  EXT8,  4,    READOP, op_ADDB},
   /* FC */   { "???  ", 1, EXT16,  5,     OTHER,       0},
   /* FD */   { "HCF  ", 1,   INH,  5,     OTHER,       0},
   /* FE */   { "LDX  ", 0, EXT16,  5,    READOP,  op_LDX},
   /* FF */   { "STX  ", 0, EXT16,  6,   WRITEOP,  op_STX}
};
