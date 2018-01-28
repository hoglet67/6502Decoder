#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <argp.h>
#include <string.h>

#include "em_6502.h"

// Sync-less decoder queue depth (samples)
// (min of 3 needed to reliably detect interrupts)
#define DEPTH 3

int sample_count = 0;

#define BUFSIZE 8192

uint16_t buffer[BUFSIZE];

// Whether to emulate each decoded instruction, to track additional state (registers and flags)
int do_emulate = 0;

#define MACHINE_DEFAULT 0
#define MACHINE_MASTER  1
#define MACHINE_ELK  2

const char *machine_names[] = {
   "default",
   "master",
   "elk",
   0
};

// ====================================================================
// Argp processing
// ====================================================================

const char *argp_program_version = "decode6502 0.1";

const char *argp_program_bug_address = "<dave@hoglet.com>";
//ndatory or optional arguments to long options are also mandatory or optional
static char doc[] = "\n\
Decoder for 6502/65C02 logic analyzer capture files.\n\
\n\
FILENAME must be a binary capture file with 16 bit samples.\n\
\n\
If FILENAME is omitted, stdin is read instead.\n\
\n\
The default bit assignments for the input signals are:\n\
 - data: bit  0 (assumes 8 consecutive bits)\n\
 -  rnw: bit  8\n\
 - sync: bit  9\n\
 -  rdy: bit 10\n\
 - phi2: bit 11\n\
 -  rst: bit 14\n\
\n\
To specify that an input is unconnected, include the option with an empty\n\
BITNUM. e.g. --sync=\n\
\n\
If phi2 is not connected the capture file should contain one sample per\n\
falling edge of phi2.\n\
\n\
If rdy is not connected a value of '1' is assumed.\n\
\n\
If sync is not connected a heuristic based decoder is used. This works well,\n\
but can take several instructions to lock onto the instruction stream.\n\
Use of sync, is preferred.\n\
";

static char args_doc[] = "[FILENAME]";

static struct argp_option options[] = {
   { "data",           1, "BITNUM",                   0, "The start bit number for data"},
   { "rnw",            2, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rnw"},
   { "sync",           3, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for sync, blank if unconnected"},
   { "rdy",            4, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rdy, blank if unconnected"},
   { "phi2",           5, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for phi2, blank if unconnected"},
   { "rst",            6, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rst, blank if unconnected"},
   { "machine",      'm', "MACHINE",                  0, "Enable machine specific behaviour"},
   { "state",        's',        0,                   0, "Show register/flag state."},
   { "hex",          'h',        0,                   0, "Show hex bytes of instruction."},
   { "cycles",       'y',        0,                   0, "Show number of bus cycles."},
   { "c02",          'c',        0,                   0, "Enable 65C02 mode."},
   { "undocumented", 'u',        0,                   0, "Enable undocumented 6502 opcodes (currently incomplete)"},
   { "byte",         'b',        0,                   0, "Byte samples"},
   { "debug",        'd',  "LEVEL",                   0, "Sets debug level (0 1 or 2)"},
   { 0 }
};

struct arguments {
   int idx_data;
   int idx_rnw;
   int idx_sync;
   int idx_rdy;
   int idx_phi2;
   int idx_rst;
   int machine;
   int show_state;
   int show_cycles;
   int show_hex;
   int c02;
   int undocumented;
   int byte;
   int debug;
   char *filename;
} arguments;

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
   int i;
   struct arguments *arguments = state->input;
   switch (key) {
   case   1:
      arguments->idx_data = atoi(arg);
   case   2:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rnw = atoi(arg);
      } else {
         arguments->idx_rnw = -1;
      }
      break;
   case   3:
      if (arg && strlen(arg) > 0) {
         arguments->idx_sync = atoi(arg);
      } else {
         arguments->idx_sync = -1;
      }
      break;
   case   4:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rdy = atoi(arg);
      } else {
         arguments->idx_rdy = -1;
      }
      break;
   case   5:
      if (arg && strlen(arg) > 0) {
         arguments->idx_phi2 = atoi(arg);
      } else {
         arguments->idx_phi2 = -1;
      }
      break;
   case   6:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rst = atoi(arg);
      } else {
         arguments->idx_rst = -1;
      }
      break;
   case 'c':
      if (arguments->undocumented) {
         argp_error(state, "undocumented and c02 flags mutually exclusive");
      }
      arguments->c02 = 1;
      break;
   case 'm':
      i = 0;
      while (machine_names[i]) {
         if (strcasecmp(arg, machine_names[i]) == 0) {
            arguments->machine = i;
            return 0;
         }
         i++;
      }
      argp_error(state, "unsupported machine type");
      break;
   case 'd':
      arguments->debug = atoi(arg);
      break;
   case 'h':
      arguments->show_hex = 1;
      break;
   case 's':
      arguments->show_state = 1;
      break;
   case 'y':
      arguments->show_cycles = 1;
      break;
   case 'u':
      if (arguments->c02) {
         argp_error(state, "undocumented and c02 flags mutually exclusive");
      }
      arguments->undocumented = 1;
      break;
   case ARGP_KEY_ARG:
      arguments->filename = arg;
      break;
   case ARGP_KEY_END:
      if (state->arg_num > 1) {
         argp_error(state, "multiple capture file arguments");
      }
      break;
   default:
      return ARGP_ERR_UNKNOWN;
   }
   return 0;
}

static struct argp argp = { options, parse_opt, args_doc, doc, 0, 0, 0 };

// ====================================================================
// Analyze a complete instruction
// ====================================================================

// TODO: all the pc prediction stuff could be pushed down into the emulation

// Predicted PC value
int pc = -1;

static void analyze_instruction(int opcode, int op1, int op2, uint64_t accumulator, int intr_seen, int num_cycles, int rst_seen) {

   int offset;
   char target[16];

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   // For instructions that push the current address to the stack we
   // can use the stacked address to determine the current PC
   int newpc = -1;
   if (rst_seen || (intr_seen && opcode != 0x00)) {
      // IRQ/NMI/RST
      newpc = (accumulator >> 24) & 0xffff;
   } else if (opcode == 0x20) {
      // JSR
      newpc = ((accumulator >> 8) - 2) & 0xffff;
   } else if (opcode == 0x00) {
      // BRK
      newpc = ((accumulator >> 24) - 2) & 0xffff;
   }

   // Sanity check the current pc prediction has not gone awry
   if (newpc >= 0) {
      if (pc >= 0 && pc != newpc) {
         printf("pc: prediction failed at %04X old pc was %04X\n", newpc, pc);
         pc = newpc;
      }
   }

   if (pc < 0) {
      printf("???? : ");
   } else {
      printf("%04X : ", pc);
   }

   int numchars = 0;
   if (rst_seen) {
      // Annotate a reset
      if (arguments.show_hex) {
         printf("         : ");
      }
      numchars = printf("RESET !!");
      if (do_emulate) {
         em_reset();
      }
   } else if (intr_seen && opcode != 0) {
      // Annotate an interrupt
      if (arguments.show_hex) {
         printf("         : ");
      }
      numchars = printf("INTERRUPT !!");
      if (do_emulate) {
         em_interrupt((accumulator >> 16) & 0xff);
      }
   } else {
      if (arguments.show_hex) {
         if (instr->len == 1) {
            printf("%02X       : ", opcode);
         } else if (instr->len == 2) {
            printf("%02X %02X    : ", opcode, op1);
         } else {
            printf("%02X %02X %02X : ", opcode, op1, op2);
         }
      }
      // Annotate a normal instruction
      const char *mnemonic = instr->mnemonic;
      const char *fmt = instr->fmt;
      switch (instr->mode) {
      case IMP:
      case IMPA:
         numchars = printf(fmt, mnemonic);
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
            sprintf(target, "%04X", pc + 2 + offset);
         }
         numchars = printf(fmt, mnemonic, target);
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
            sprintf(target, "%04X", pc + 3 + offset);
         }
         numchars = printf(fmt, mnemonic, op1, target);
         break;
      case IMM:
      case ZP:
      case ZPX:
      case ZPY:
      case INDX:
      case INDY:
      case IND:
         numchars = printf(fmt, mnemonic, op1);
         break;
      case ABS:
      case ABSX:
      case ABSY:
      case IND16:
      case IND1X:
         numchars = printf(fmt, mnemonic, op1, op2);
         break;
      }

      if ((arguments.show_cycles || (arguments.show_state))) {
         // Pad opcode to 14 characters, to match python
         while (numchars++ < 14) {
            printf(" ");
         }
      }

      // Emulate the instruction
      if (do_emulate) {
         if (instr->emulate) {
            int operand;
            if (instr->optype == RMWOP) {
               // e.g. <opcode> <op1> <op2> <read> <write> <write>
               // Want to pick off the read
               operand = (accumulator >> 16) & 0xff;
            } else if (instr->optype == BRANCHOP) {
               // the operand is true if branch taken
               operand = (num_cycles != 2);
            } else if (opcode == 0x40) {
               // RTI: the operand (flags) is the first read cycle of three
               operand = (accumulator >> 16) & 0xff;
            } else if (instr->mode == IMM) {
               // Immediate addressing mode: the operand is the 2nd byte of the instruction
               operand = op1;
            } else if (instr->decimalcorrect && (em_get_D() == 1)) {
               // read operations on the C02 that have an extra cycle added
               operand = (accumulator >> 8) & 0xff;
            } else if (instr->optype == TSBTRBOP) {
               // For TSB/TRB, <opcode> <op1> <read> <dummy> <write> the operand is the <read>
               operand = (accumulator >> 16) & 0xff;
            } else {
               // default to using the last bus cycle as the operand
               operand = accumulator & 0xff;
            }
            printf(" : op=%02x", operand);
            instr->emulate(operand);
         }
      }
   }

   if (arguments.show_cycles) {
      printf(" : %d", num_cycles);
   }

   if (arguments.show_state) {
      printf(" : %s", em_get_state());
   }

   printf("\n");

   // Look for control flow changes and update the PC
   if (opcode == 0x40 || opcode == 0x00 || opcode == 0x6c || opcode == 0x7c || intr_seen || rst_seen) {
      // RTI, BRK, INTR, JMP (ind), JMP (ind, X), IRQ/NMI/RST
      pc = ((accumulator & 0xFF00) >> 8) | ((accumulator & 0x00FF) << 8);
   } else if (opcode == 0x20 || opcode == 0x4c) {
      // JSR abs, JMP abs
      pc = op2 << 8 | op1;
   } else if (opcode == 0x60) {
      // RTS
      pc = ((((accumulator & 0xff0000) >> 16) | (accumulator & 0xff00)) + 1) & 0xffff;
   } else if (pc < 0) {
      // PC value is not known yet, everything below this point is relative
      pc = -1;
   } else if (opcode == 0x80) {
      // BRA
      pc += ((int8_t)(op1)) + 2;
      pc &= 0xffff;
   } else if (((opcode & 0x0f) == 0x0f) && (num_cycles != 5)) {
      // BBR/BBS: op2 if taken
      pc += ((int8_t)(op2)) + 3;
      pc &= 0xffff;
   } else if ((opcode & 0x1f) == 0x10 && num_cycles != 2) {
      // BXX: op1 if taken
      pc += ((int8_t)(op1)) + 2;
      pc &= 0xffff;
   } else {
      // Otherwise, increment pc by length of instuction
      pc += instr->len;
      pc &= 0xffff;
   }
}

// ====================================================================
// Sync-less bus cycle decoder
// ====================================================================

void decode_cycle_without_sync(int *bus_data_q, int *pin_rnw_q, int *pin_rst_q) {

   // Count of the 6502 bus cycles
   static int cyclenum             = 0;

   // Cycle count of the last sync, so we know the instruction cycle count
   static int last_cyclenum        = 0;

   // State to decode the 6502 bus activity
   static int opcode               = -1;
   static int op1                  = 0;
   static int op2                  = 0;
   static uint64_t accumulator     = 0;
   static int bus_cycle            = 0;
   static int cycle_count          = 0;
   static int opcount              = 0;
   static int rst_seen             = 0;
   static int intr_seen            = 0;

   int bus_data = *bus_data_q;
   int pin_rnw = *pin_rnw_q;
   int pin_rst = *pin_rst_q;

   // Detect a rising edge of reset
   if (arguments.idx_rst >= 0) {
      if (pin_rst == 0) {
         if (*(pin_rst_q + 1) == 1) {
            cycle_count = 8;
            bus_cycle = -1;
            opcode = 0;
            rst_seen = 1;
         }
         return;
      }
   } else {
      // no reset line
      if ((bus_data_q[0] == 0xCD) && (bus_data_q[1] == 0xD9) && (bus_data_q[2] == 0xA9)) {
         // reset vector answer B 1.20 OS + LDA#
         cycle_count = 1;
         bus_cycle = -1;
         opcode = 0;
         rst_seen = 1;
      }
   }

   // TODO: Find a more correct way of starting up!
   InstrType *instr = &instr_table[opcode >= 0 ? opcode : 0xEA];

   if (bus_cycle == 1 && opcount >= 1) {
      op1 = bus_data;
   }

   if (bus_cycle == ((opcode == 0x20) ? 5 : ((opcode & 0x0f) == 0x0f) ? 4 : 2) && opcount >= 2) {
      op2 = bus_data;
   }

   // Account for extra cycle in ADC/SBC in decimal mode in C02
   if (arguments.c02 && instr->decimalcorrect && bus_cycle == 1 && em_get_D() == 1) {
      cycle_count++;
   }

   // Account for extra cycle in a page crossing in (indirect), Y
   if (bus_cycle == 4) {
      // Applies to INDY, but need to exclude stores
      if ((instr->mode == INDY) && (instr->optype == READOP)) {
         int index = em_get_Y();
         if (index >= 0) {
            int base = ((accumulator & 0xFF00) >> 8) | ((accumulator & 0x00FF) << 8);
            if ((base & 0xff00) != ((base + index) & 0xff00)) {
               cycle_count++;
            }
         }
      }
   }

   // Account for extra cycle in a page crossing in absolute indexed
   if (bus_cycle == 2) {
      // Applies to ABSX and ABSY, but need to exclude stores
      if (((instr->mode == ABSX) || (instr->mode == ABSY)) && (instr->optype == READOP)) {
         // 6502:  Need to exclude ASL/ROL/LSR/ROR/DEC/INC, which are 7 cycles regardless
         // 65C02: Need to exclude DEC/INC, which are 7 cycles regardless
         if ((opcode != 0xDE) && (opcode != 0xFE) && (arguments.c02 || ((opcode != 0x1E) && (opcode != 0x3E) && (opcode != 0x5E) && (opcode != 0x7E)))) {
            int index = (instr->mode == ABSX) ? em_get_X() : em_get_Y();
            if (index >= 0) {
               int base = op1 + (op2 << 8);
               if ((base & 0xff00) != ((base + index) & 0xff00)) {
                  cycle_count++;
               }
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

   if (bus_cycle == 4) {
      if ((opcode & 0x0f) == 0x0f) {
         int operand = (accumulator >> 8) & 0xff;
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
            if (pc >= 0) {
               int target =  (pc + 3) + ((int8_t)(op2));
               if ((target & 0xFF00) != ((pc + 3) & 0xff00)) {
                  cycle_count = 7;
               }
            }
         }
      }
   }

   // Account for extra cycles in a branch
   if (bus_cycle == 1) {
      if (((opcode & 0x1f) == 0x10) || (opcode == 0x80)) {
         // Default to backards branches taken, forward not taken
         int taken = ((int8_t)op1) < 0;
         switch (opcode) {
         case 0x10: // BPL
            if (em_get_N() >= 0) {
               taken = !em_get_N();
            }
            break;
         case 0x30: // BMI
            if (em_get_N() >= 0) {
               taken = em_get_N();
            }
            break;
         case 0x50: // BVC
            if (em_get_V() >= 0) {
               taken = !em_get_V();
            }
            break;
         case 0x70: // BVS
            if (em_get_V() >= 0) {
               taken = em_get_V();
            }
            break;
         case 0x80: // BRA
            taken = 1;
            break;
         case 0x90: // BCC
            if (em_get_C() >= 0) {
               taken = !em_get_C();
            }
            break;
         case 0xB0: // BCS
            if (em_get_C() >= 0) {
               taken = em_get_C();
            }
            break;
         case 0xD0: // BNE
            if (em_get_Z() >= 0) {
               taken = !em_get_Z();
            }
            break;
         case 0xF0: // BEQ
            if (em_get_Z() >= 0) {
               taken = em_get_Z();
            }
            break;
         }
         if (taken) {
            // A taken branch is 3 cycles, not 2
            cycle_count = 3;
            // A taken branch that crosses a page boundary is 4 cycle
            if (pc >= 0) {
               int target =  (pc + 2) + ((int8_t)(op1));
               if ((target & 0xFF00) != ((pc + 2) & 0xff00)) {
                  cycle_count = 4;
               }
            }
         }
      }
   }

   // Master specific behaviour to remain in sync if rdy is not available
   if ((arguments.machine == MACHINE_MASTER) && (arguments.idx_rdy < 0)) {
      static int mhz1_phase = 0;
      if ((bus_cycle == 3) && (instr->len == 3)) {
         if ((op2 == 0xfc) ||                 // &FC00-&FCFF
             (op2 == 0xfd) ||                 // &FD00-&FDFF
             (op2 == 0xfe && (
                ((op1 & 0xE0) == 0x00) ||     // &FE00-&FE1F
                ((op1 & 0xC0) == 0x40) ||     // &FE40-&FE7F
                ((op1 & 0xE0) == 0x80) ||     // &FE80-&FE9F
                ((op1 & 0xE0) == 0xC0)        // &FEC0-&FEDF
                ))) {
            // Use STA/STX/STA to determine 1MHz clock phase
            if (opcode == 0x8C || opcode == 0x8D || opcode == 0x8E) {
               if (*(bus_data_q + 1) == bus_data) {
                  int new_phase;
                  if (*(bus_data_q + 2) == bus_data) {
                     new_phase = 1;
                  } else {
                     new_phase = 0;
                  }
                  if (mhz1_phase != new_phase) {
                     printf("correcting 1MHz phase\n");
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
      mhz1_phase = 1 - mhz1_phase;
   }

   // An interrupt sequence looks like:
   // 0 <opcode>            Rd
   // 1 <opcode>            Rd // address not incremented
   // 2 <return address hi> Wr
   // 3 <return address lo> Wr
   // 4 <flags>             Wr
   // 5 <vector lo>         Rd
   // 6 <vector hu>         Rd
   // 7 <opcode>            Rd

   // Detect interrupts as early as possible...
   if (arguments.idx_rnw >= 0) {
      if ((bus_cycle == 2) && (pin_rnw == 0) && (*(pin_rnw_q + 1) == 0) && (*(pin_rnw_q + 2) == 0)) {
         cycle_count = 7;
         intr_seen = 1;
      }
   } else {
      if ((bus_cycle == 2) && (bus_data_q[0] == ((pc >> 8) & 0xff)) && (bus_data_q[1] == (pc & 0xff))) {
         cycle_count = 7;
         intr_seen = 1;
      }
   }

   if (bus_cycle == cycle_count) {
      // Analyze the  previous instrucution
      if (opcode >= 0) {
         analyze_instruction(opcode, op1, op2, accumulator, intr_seen, cyclenum - last_cyclenum, rst_seen);
         rst_seen = 0;
         intr_seen = 0;
      }
      last_cyclenum  = cyclenum;

      // Re-initialize the state for the new instruction
      opcode            = bus_data;
      op1               = 0;
      op2               = 0;
      accumulator       = 0;
      bus_cycle         = 0;
      cycle_count       = instr_table[opcode].cycles ;
      opcount           = instr_table[opcode].len - 1;

   } else {

      accumulator = (accumulator << 8) | bus_data;

   }

   if (arguments.debug >= 1) {
      printf("%d %02x", bus_cycle, bus_data);
      if (arguments.idx_rnw >= 0) {
         printf(" %d", pin_rnw);
      }
      if (arguments.idx_rst >= 0) {
         printf(" %d", pin_rst);
      }
      printf("\n");
   }

   bus_cycle++;

   // Increment the cycle number (used only to detect taken branches)
   cyclenum++;
}

void lookahead_decode_cycle_without_sync(int bus_data, int pin_rnw, int pin_rst) {
   static int bus_data_q[DEPTH];
   static int pin_rnw_q[DEPTH];
   static int pin_rst_q[DEPTH];
   static int fill = 0;

   bus_data_q[fill] = bus_data;
   pin_rnw_q[fill] = pin_rnw;
   pin_rst_q[fill] = pin_rst;
   if (fill < DEPTH - 1) {
      fill++;
   } else {
      decode_cycle_without_sync(bus_data_q, pin_rnw_q, pin_rst_q);
      for (int i = 0; i < DEPTH - 1; i++) {
         bus_data_q[i] = bus_data_q[i + 1];
         pin_rnw_q[i] = pin_rnw_q[i + 1];
         pin_rst_q[i] = pin_rst_q[i + 1];
      }
   }
}

// ====================================================================
// Sync-based bus cycle decoder
// ====================================================================

void decode_cycle_with_sync(int bus_data, int pin_rnw, int pin_sync, int pin_rst) {
#if 0

   // Count of the 6502 bus cycles
   static int cyclenum             = 0;

   // Cycle count of the last sync, so we know the instruction cycle count
   static int last_cyclenum        = 0;

   // State to decode the 6502 bus activity
   static int opcode               = -1;
   static int opcount              = 0;
   static int op1                  = 0;
   static int op2                  = 0;
   static int bus_cycle            = 0;
   static int write_count          = 0;
   static int read_accumulator     = 0;
   static int write_accumulator    = 0;
   static int last_pin_rst         = 1;
   static int rst_seen             = 0;

   if (pin_rst == 1) {

      if (last_pin_rst == 0) {
         rst_seen = 1;
         opcode = -1;
      }

      if (pin_sync == 1) {

         // Sync indicates the start of a new instruction, the following variables pertain to the previous instruction
         // opcode, op1, op2, read_accumulator, write_accumulator, write_count, operand

         // Analyze the  previous instrucution
         if (opcode >= 0) {
            analyze_instruction(opcode, op1, op2, read_accumulator, write_accumulator, write_count == 3, cyclenum - last_cyclenum, rst_seen);
            rst_seen = 0;
         }
         last_cyclenum  = cyclenum;

         bus_cycle         = 0;
         opcode            = bus_data;
         opcount           = instr_table[opcode].len - 1;
         write_count       = 0;
         read_accumulator  = 0;
         write_accumulator = 0;

      } else if (pin_rnw == 0) {
         if (bus_cycle == 2 || bus_cycle == 3 || bus_cycle == 4) {
            write_count++;
         }
         write_accumulator = (write_accumulator << 8) | bus_data;

      } else if (bus_cycle == 1 && opcount >= 1) {
         op1 = bus_data;

      } else if (bus_cycle == ((opcode == 0x20) ? 5 : ((opcode & 0x0f) == 0x0f) ? 4 : 2) && opcount >= 2 && write_count < 3) {
         // JSR     is <opcode> <op1> <dummp stack rd> <stack wr> <stack wr> <op2>
         // BBR/BBS is <opcode> <op1> <zp> <dummy> <op2> (<branch taken penalty>) (<page cross penatly)
         op2 = bus_data;

      } else {
         read_accumulator = (read_accumulator <<  8) | bus_data;
      }

      if (arguments.debug >= 1) {
         printf("%d %02x %d %d %d\n", bus_cycle, bus_data, pin_rnw, pin_sync, pin_rst);
      }

      bus_cycle++;
   }

   // Increment the cycle number (used only to detect taken branches)
   cyclenum++;

   // Maintain an edge detector for rst
   last_pin_rst = pin_rst;
#endif
}

// ====================================================================
// Input file processing and bus cycle extraction
// ====================================================================

void decode(FILE *stream) {

   // Pin mappings into the 16 bit words
   int idx_data  = arguments.idx_data;
   int idx_rnw   = arguments.idx_rnw ;
   int idx_sync  = arguments.idx_sync;
   int idx_rdy   = arguments.idx_rdy ;
   int idx_phi2  = arguments.idx_phi2;
   int idx_rst   = arguments.idx_rst;

   // Default Pin values
   int bus_data  =  0;
   int pin_rnw   =  1;
   int pin_sync  =  0;
   int pin_rdy   =  1;
   int pin_phi2  =  0;
   int pin_rst   =  1;

   int num;

   // The previous sample of the 16-bit capture (async sampling only)
   uint16_t sample       = -1;
   uint16_t last_sample  = -1;
   uint16_t last2_sample = -1;

   // The previous sample of phi2 (async sampling only)
   int last_phi2 = -1;

   while ((num = fread(buffer, sizeof(uint16_t), BUFSIZE, stream)) > 0) {

      uint16_t *sampleptr = &buffer[0];

      while (num-- > 0) {

         // The current 16-bit capture sample, and the previous two
         last2_sample = last_sample;
         last_sample  = sample;
         sample       = *sampleptr++;

         // TODO: fix the hard coded values!!!
         if (arguments.debug >= 2) {
            printf("%d %02x %x %x %x %x\n", sample_count, sample&255, (sample >> 8)&1,  (sample >> 9)&1,  (sample >> 10)&1,  (sample >> 11)&1  );
         }
         sample_count++;

         // Phi2 is optional
         // - if asynchronous capture is used, it must be connected
         // - if synchronous capture is used, it must not connected
         if (idx_phi2 < 0) {

            // If Phi2 is not present, use the pins directly
            bus_data = (sample >> idx_data) & 255;
            if (idx_rnw >= 0) {
               pin_rnw = (sample >> idx_rnw ) & 1;
            }
            if (idx_sync >= 0) {
               pin_sync = (sample >> idx_sync) & 1;
            }
            if (idx_rdy >= 0) {
               pin_rdy = (sample >> idx_rdy) & 1;
            }
            if (idx_rst >= 0) {
               pin_rst = (sample >> idx_rst) & 1;
            }

         } else {

            // If Phi2 is present, look for an edge
            pin_phi2 = (sample >> idx_phi2) & 1;
            if (pin_phi2 == last_phi2) {
               // continue for more samples
               continue;
            }
            last_phi2 = pin_phi2;

            if (pin_phi2) {
               // sample control signals just after rising edge of Phi2
               if (idx_rnw >= 0) {
                  pin_rnw = (sample >> idx_rnw ) & 1;
               }
               if (idx_sync >= 0) {
                  pin_sync = (sample >> idx_sync) & 1;
               }
               if (idx_rst >= 0) {
                  pin_rst = (sample >> idx_rst) & 1;
               }
               // continue for more samples
               continue;
            } else {
               if (idx_rdy >= 0) {
                  pin_rdy = (last_sample >> idx_rdy) & 1;
               }
               // TODO: try to rationalize this!
               if (arguments.machine == MACHINE_ELK) {
                  // Data bus sampling for the Elk
                  if (idx_rnw < 0 || pin_rnw) {
                     // sample read data just before falling edge of Phi2
                     bus_data = last_sample & 255;
                  } else {
                     // sample write data one cycle earlier
                     bus_data = last_sample & 255;
                  }
               } else if (arguments.machine == MACHINE_MASTER) {
                  // Data bus sampling for the Master
                  if (idx_rnw < 0 || pin_rnw) {
                     // sample read data just before falling edge of Phi2
                     bus_data = last_sample & 255;
                  } else {
                     // sample write data one cycle earlier
                     bus_data = last2_sample & 255;
                  }
               } else {
                  // Data bus sampling for the Beeb, one cycle later
                  if (idx_rnw < 0 || pin_rnw) {
                     // sample read data just after falling edge of Phi2
                     bus_data = sample & 255;
                  } else {
                     // sample write data one cycle earlier
                     bus_data = last_sample & 255;
                  }
               }
            }
         }

         // Ignore the cycle if RDY is low
         if (pin_rdy == 0)
            continue;

         if (idx_sync < 0) {
            lookahead_decode_cycle_without_sync(bus_data, pin_rnw, pin_rst);
         } else {
            decode_cycle_with_sync(bus_data, pin_rnw, pin_sync, pin_rst);
         }
      }
   }
}

// ====================================================================
// Main program entry point
// ====================================================================

int main(int argc, char *argv[]) {
   arguments.idx_data     =  0;
   arguments.idx_rnw      =  8;
   arguments.idx_sync     =  9;
   arguments.idx_rdy      = 10;
   arguments.idx_phi2     = 11;
   arguments.idx_rst      = 14;
   arguments.machine      = MACHINE_DEFAULT;
   arguments.show_hex     = 0;
   arguments.show_state   = 0;
   arguments.show_cycles  = 0;
   arguments.c02          = 0;
   arguments.undocumented = 0;
   arguments.byte         = 1;
   arguments.debug        = 0;
   arguments.filename     = NULL;

   argp_parse(&argp, argc, argv, 0, 0, &arguments);

   if (arguments.show_state || arguments.idx_sync < 0) {
      do_emulate = 1;
   }

   FILE *stream;
   if (!arguments.filename || !strcmp(arguments.filename, "-")) {
      stream = stdin;
   } else {
      stream = fopen(arguments.filename, "r");
      if (stream == NULL) {
         perror("failed to open capture file");
         return 2;
      }
   }
   em_init(arguments.c02, arguments.undocumented);
   decode(stream);
   fclose(stream);
   return 0;
}
