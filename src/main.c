#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <argp.h>
#include <string.h>

#include "em_6502.h"
#include "profiler.h"

// Sync-less decoder queue depth (samples)
// (min of 3 needed to reliably detect interrupts)
#define DEPTH 3

int sample_count = 0;

#define BUFSIZE 8192

uint8_t buffer8[BUFSIZE];

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
\n\
If RST is not connected, an alternative is to specify the reset vector:\n\
 - D9CD (D9 is the high byte, CD is the low byte)\n\
 - A9D9CD (optionally, also specify the first opcode, LDA # in this case)\n\
\n\
";

static char args_doc[] = "[FILENAME]";

static struct argp_option options[] = {
   { "data",           1, "BITNUM",                   0, "The start bit number for data"},
   { "rnw",            2, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rnw"},
   { "sync",           3, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for sync, blank if unconnected"},
   { "rdy",            4, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rdy, blank if unconnected"},
   { "phi2",           5, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for phi2, blank if unconnected"},
   { "rst",            6, "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for rst, blank if unconnected"},
   { "vecrst",         7,    "HEX", OPTION_ARG_OPTIONAL, "The reset vector, black if not known"},
   { "machine",      'm', "MACHINE",                  0, "Enable machine specific behaviour"},
   { "emulate",      'e',        0,                   0, "Enable emulation, for error checking."},
   { "c02",          'c',        0,                   0, "Enable 65C02 mode."},
   { "rockwell",     'r',        0,                   0, "Enable additional rockwell instructions."},
   { "undocumented", 'u',        0,                   0, "Enable undocumented 6502 opcodes (currently incomplete)"},
   { "byte",         'b',        0,                   0, "Byte samples"},
   { "debug",        'd',  "LEVEL",                   0, "Sets debug level (0 1 or 2)"},
// Output options
   { "quiet",        'q',        0,                   0, "Set all the show options to off."},
   { "address",      'a',        0,                   0, "Show address of instruction."},
   { "hex",          'h',        0,                   0, "Show hex bytes of instruction."},
   { "instruction",  'i',        0,                   0, "Show instruction."},
   { "state",        's',        0,                   0, "Show register/flag state."},
   { "cycles",       'y',        0,                   0, "Show number of bus cycles."},
   { "profile",      'p', "PARAMS", OPTION_ARG_OPTIONAL, "Profile code execution."},
   { "trigger",      't',"ADDRESS",                   0, "Trigger on address."},
   { "bbcfwa",       'f',        0,                   0, "Show BBC floating poing work areas."},
   { "bbctube",       8,         0,                   0, "Decode BBC tube protocol"},

   { 0 }
};

struct arguments {
   int idx_data;
   int idx_rnw;
   int idx_sync;
   int idx_rdy;
   int idx_phi2;
   int idx_rst;
   int vec_rst;
   int machine;
   int show_address;
   int show_hex;
   int show_instruction;
   int show_state;
   int show_bbcfwa;
   int show_cycles;
   int show_something;
   int bbctube;
   int emulate;
   int c02;
   int rockwell;
   int undocumented;
   int byte;
   int debug;
   int profile;
   int trigger_start;
   int trigger_stop;
   int trigger_skipint;
   char *filename;
} arguments;

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
   int i;
   struct arguments *arguments = state->input;

   // First, pass argument to the profiler moduler
   profiler_parse_opt(key, arg, state);

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
   case   7:
      if (arg && strlen(arg) > 0) {
         arguments->vec_rst = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->vec_rst = -1;
      }
      break;
   case   8:
      arguments->bbctube = 1;
      break;
   case 'c':
      if (arguments->undocumented) {
         argp_error(state, "undocumented and c02 flags mutually exclusive");
      }
      arguments->c02 = 1;
      break;
   case 'r':
      arguments->rockwell = 1;
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
   case 'b':
      arguments->byte = 1;
      break;
   case 'q':
      arguments->show_address = 0;
      arguments->show_hex = 0;
      arguments->show_instruction = 0;
      arguments->show_state = 0;
      arguments->show_bbcfwa = 0;
      arguments->show_cycles = 0;
      break;
   case 'a':
      arguments->show_address = 1;
      break;
   case 'h':
      arguments->show_hex = 1;
      break;
   case 'i':
      arguments->show_instruction = 1;
      break;
   case 's':
      arguments->show_state = 1;
      break;
   case 'f':
      arguments->show_bbcfwa = 1;
      break;
   case 'y':
      arguments->show_cycles = 1;
      break;
   case 'e':
      arguments->emulate = 1;
      break;
   case 'p':
      arguments->profile = 1;
      break;
   case 't':
      if (arg && strlen(arg) > 0) {
         char *start   = strtok(arg, ",");
         char *stop    = strtok(NULL, ",");
         char *skipint = strtok(NULL, ",");
         if (start && strlen(start) > 0) {
            arguments->trigger_start = strtol(start, (char **)NULL, 16);
         }
         if (stop && strlen(stop) > 0) {
            arguments->trigger_stop = strtol(stop, (char **)NULL, 16);
         }
         if (skipint && strlen(skipint) > 0) {
            arguments->trigger_skipint = atoi(skipint);
         }
      }
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

   static int interrupt_depth = 0;
   static int skipping_interrupted = 0;
   static int triggered = 0;

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

   // Force the pc to don't case fore the reset cycle (makes test log more consistent)
   if (rst_seen) {
      pc = -1;
   }

   if (rst_seen) {
      // Handlea reset
      if (do_emulate) {
         em_reset();
      }
   } else if (intr_seen && opcode != 0) {
      // Handle an interrupt
      if (do_emulate) {
         em_interrupt((accumulator >> 16) & 0xff, pc);
      }
   } else {
      // Handle a normal instruction
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
            } else if (opcode == 0x00) {
               // BRK: the operand is the data pushed to the stack (PCH, PCL, P)
               // <opcode> <op1> <write pch> <write pcl> <write p> <read rst> <read rsth>
               operand = (accumulator >> 16) & 0xffffff;
            } else if (opcode == 0x20) {
               // JSR: the operand is the data pushed to the stack (PCH, PCL)
               // <opcode> <op1> <read dummy> <write pch> <write pcl> <op2>
               operand = (accumulator >> 8) & 0xffff;
            } else if (opcode == 0x40) {
               // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
               // <opcode> <op1> <read dummy> <read p> <read pcl> <read pch>
               operand = accumulator & 0xffffff;
            } else if (opcode == 0x60) {
               // RTS: the operand is the data pulled from the stack (PCL, PCH)
               // <opcode> <op1> <read dummy> <read pcl> <read pch>
               operand = (accumulator >> 8) & 0xffff;
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
               index = instr->mode == ZPX ? em_get_X() : em_get_Y();
               if (index >= 0) {
                  ea = (op1 + index) & 0xff;
               }
               break;
            case INDY:
               // <opcpde> <op1> <addrlo> <addrhi> [ <page crossing>] <<operand> [ <extra cycle in dec mode> ]
               index = em_get_Y();
               if (index >= 0) {
                  ea = accumulator >> (8 * (num_cycles - 4));
                  ea = ((ea & 0xFF00) >> 8) | ((ea & 0x00FF) << 8);
                  ea = (ea + index) & 0xffff;
               }
               break;
            case INDX:
               // <opcpde> <op1> <dummy> <addrlo> <addrhi> <operand> [ <extra cycle in dec mode> ]
               ea = accumulator >> (8 * (num_cycles - 5));
               ea = ((ea & 0xFF00) >> 8) | ((ea & 0x00FF) << 8);
               break;
            case IND:
               // <opcpde> <op1> <addrlo> <addrhi> <operand> [ <extra cycle in dec mode> ]
               ea = accumulator >> (8 * (num_cycles - 4));
               ea = ((ea & 0xFF00) >> 8) | ((ea & 0x00FF) << 8);
               break;
            case ABS:
               ea = op2 << 8 | op1;
               break;
            case ABSX:
            case ABSY:
               index = instr->mode == ABSX ? em_get_X() : em_get_Y();
               if (index >= 0) {
                  ea = ((op2 << 8 | op1) + index) & 0xffff;
               }
               break;
            default:
               break;
            }

            instr->emulate(operand, ea);
         }
      }
   }

   if (arguments.trigger_start < 0 || (pc >= 0 && pc == arguments.trigger_start)) {
      triggered = 1;
   } else if (pc >= 0 && pc == arguments.trigger_stop) {
      triggered = 0;
   }

   // Exclude interrupts from profiling
   if (arguments.trigger_skipint && pc >= 0) {
      if (interrupt_depth == 0) {
         skipping_interrupted = 0;
      }
      if (intr_seen && opcode != 0) {
         interrupt_depth++;
         skipping_interrupted = 1;
      } else if (interrupt_depth > 0 && opcode == 0x40) {
         interrupt_depth--;
      }
   }

   if (arguments.profile && triggered && !skipping_interrupted && (!intr_seen || opcode == 0)) {
      profiler_profile_instruction(pc, opcode, op1, op2, num_cycles);
   }

   int fail = em_get_and_clear_fail();

   if ((fail | arguments.show_something) && triggered && !skipping_interrupted) {
      int numchars = 0;
      // Show address
      if (fail || arguments.show_address) {
         if (pc < 0) {
            printf("???? : ");
         } else {
            printf("%04X : ", pc);
         }
      }
      // Show hex bytes
      if (fail || arguments.show_hex) {
         if (rst_seen) {
            printf("         : ");
         } else if (intr_seen && opcode != 0) {
            printf("         : ");
         } else {
            if (instr->len == 1) {
               printf("%02X       : ", opcode);
            } else if (instr->len == 2) {
               printf("%02X %02X    : ", opcode, op1);
            } else {
               printf("%02X %02X %02X : ", opcode, op1, op2);
            }
         }
      }
      // Show instruction disassembly
      if (fail || arguments.show_something) {
         if (rst_seen) {
            numchars = printf("RESET !!");
         } else if (intr_seen && opcode != 0) {
            numchars = printf("INTERRUPT !!");
         } else {
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
                  sprintf(target, "%04X", (pc + 2 + offset) & 0xffff);
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
                  sprintf(target, "%04X", (pc + 3 + offset) & 0xffff);
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
         }
      }
      // Pad if there is more to come
      if (fail || arguments.show_cycles || arguments.show_state || arguments.show_bbcfwa) {
         // Pad opcode to 14 characters, to match python
         while (numchars++ < 14) {
            printf(" ");
         }
      }
      // Show cycles (don't include with fail as it is inconsistent depending on whether rdy is present)
      if (arguments.show_cycles) {
         printf(" : %d", num_cycles);
      }
      // Show register state
      if (fail || arguments.show_state) {
         printf(" : %s", em_get_state());
      }
      // Show BBC floating point work area FWA, FWB
      if (arguments.show_bbcfwa) {
         printf(" : FWA %s", em_get_fwa(0x2e, 0x30, 0x31, 0x35, 0x2f));
         printf(" : FWB %s", em_get_fwa(0x3b, 0x3c, 0x3d, 0x41,   -1));
      }
      // Show any errors
      if (fail) {
         printf(" prediction failed");
      }
      // End the line
      printf("\n");
   }

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
   } else if (arguments.c02 && arguments.rockwell && ((opcode & 0x0f) == 0x0f) && (num_cycles != 5)) {
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
   static int mhz1_phase           = 0;

   int bus_data = *bus_data_q;
   int pin_rnw = *pin_rnw_q;
   int pin_rst = *pin_rst_q;

   // Detect a rising edge of reset
   if (arguments.idx_rst >= 0) {
      if (pin_rst == 0) {
         if (*(pin_rst_q + 1) == 1) {
            cycle_count = arguments.c02 ? 7 : 8;
            bus_cycle = -1;
            opcode = 0;
            rst_seen = 1;
         }
         return;
      }
   } else {
      // no reset line
      if ((bus_data_q[0] + (bus_data_q[1] << 8)) == (arguments.vec_rst & 0xFFFF)) {
         int rst_opcode = (arguments.vec_rst >> 16) & 0xff;
         if (!rst_opcode || (bus_data_q[2] == rst_opcode)) {
            cycle_count = 1;
            bus_cycle = -1;
            opcode = 0;
            rst_seen = 1;
         }
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

   if (!intr_seen && !rst_seen) {

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
         if (arguments.c02 && arguments.rockwell && (opcode & 0x0f) == 0x0f) {
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
                        // printf("correcting 1MHz phase\n");
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
            // Now test unused flag is 1, B is 0, and all other known flags match
            if (((bus_data_q[2] & 0x30) == 0x20) && !compare_NVDIZC(bus_data_q[2])) {
               cycle_count = 7;
               intr_seen = 1;
            }
         }
      }
   }

   // Master specific behaviour to remain in sync if rdy is not available
   if ((arguments.machine == MACHINE_MASTER) && (arguments.idx_rdy < 0)) {
      // Toggle the phase every cycle
      mhz1_phase = 1 - mhz1_phase;
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
   static int rst_state            = 0;
   static int intr_state           = 0;
   static uint64_t accumulator     = 0;
   static int last_pin_rst         = 1;

   // TODO, sync based decoder probably should fall back to testing the
   // reset vector if rst is not connected.

   if (pin_rst == 1) {

      if (arguments.idx_rst >= 0) {
         // If we have the RST pin connected, we use it's value directly
         if (last_pin_rst == 0) {
            rst_state = 3;
         }
      } else {
         // If not, then we use a heuristic, based on what we expect to see on the data
         // bus in cycles 5, 6 and 7, i.e. RSTVECL, RSTVECH, RSTOPCODE
         if ((bus_cycle == 5) && (bus_data == (arguments.vec_rst & 0xff))) {
            // Matched RSTVECL
            rst_state++;
         }
         if ((bus_cycle == 6) && (bus_data == ((arguments.vec_rst >> 8) & 0xff))) {
            // Matched RSTVECH
            rst_state++;
         }
         if ((bus_cycle == 7) && ((bus_data == ((arguments.vec_rst >> 16) & 0xff)) || (((arguments.vec_rst >> 16) & 0xff) == 0))) {
            // Matched RSTOPCODE if none zerl
            rst_state++;
         }
      }

      // Write count is only used to determine if an interrupt has occurred
      if (arguments.idx_rnw >= 0) {
         // If we have the RNW pin connected, we use it's value directly
         if (pin_rnw == 0) {
            if (bus_cycle == 2 || bus_cycle == 3 || bus_cycle == 4) {
               intr_state++;
            }
         }
      } else {
         // If not, then we use a heuristic, based on what we expect to see on the data
         // bus in cycles 2, 3 and 4, i.e. PCH, PCL, PSW
         // (It's unlikely sync would be connected but rnw not, but lets try to cope)
         if ((bus_cycle == 2) && (bus_data == ((pc >> 8) & 0xff))) {
            // Matched PCH
            intr_state++;
         }
         if ((bus_cycle == 3) && (bus_data == (pc & 0xff))) {
            // Matched PCL
            intr_state++;
         }
         // Now test unused flag is 1, B is 0
         if ((bus_cycle == 4) && ((bus_data & 0x30) == 0x20)) {
            // Finally test all other known flags match
            if (!compare_NVDIZC(bus_data)) {
               // Matched PSW = NV-BDIZC
               intr_state++;
            }
         }
      }

      if (pin_sync == 1) {

         // Sync indicates the start of a new instruction, the following variables pertain to the previous instruction
         // opcode, op1, op2, accumulator, intr_state, operand

         // Analyze the  previous instrucution
         if (opcode >= 0) {
            analyze_instruction(opcode, op1, op2, accumulator, intr_state == 3, cyclenum - last_cyclenum, rst_state == 3);
            rst_state = 0;
         }
         last_cyclenum  = cyclenum;

         bus_cycle         = 0;
         opcode            = bus_data;
         opcount           = instr_table[opcode].len - 1;
         intr_state        = 0;
         accumulator       = 0;

      } else {

         if (bus_cycle == 1 && opcount >= 1) {
            op1 = bus_data;
         }

         if (bus_cycle == ((opcode == 0x20) ? 5 : ((opcode & 0x0f) == 0x0f) ? 4 : 2) && opcount >= 2 && intr_state < 3) {
            // JSR     is <opcode> <op1> <dummp stack rd> <stack wr> <stack wr> <op2>
            // BBR/BBS is <opcode> <op1> <zp> <dummy> <op2> (<branch taken penalty>) (<page cross penatly)
            op2 = bus_data;
         }

         accumulator = (accumulator <<  8) | bus_data;
      }

   } else {
      // In reset we reset the opcode back to -1. This prevents a spurious
      // instrunction being emitted at the start of the 7-cycle reset sequence.
      opcode = -1;
   }

   if (arguments.debug >= 1) {
      printf("%d %02x %d", bus_cycle, bus_data, pin_sync);
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

   // Maintain an edge detector for rst
   last_pin_rst = pin_rst;
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

   if (arguments.byte) {

      // In byte mode we have only data bus samples, nothing else so we must
      // use the sync-less decoder. The values of pin_rnw and pin_rst are set
      // to 1, but the decoder should never actually use them.

      while ((num = fread(buffer8, sizeof(uint8_t), BUFSIZE, stream)) > 0) {
         uint8_t *sampleptr = &buffer8[0];
         while (num-- > 0) {
            lookahead_decode_cycle_without_sync(*sampleptr++, 1, 1);
         }
      }

   } else {

      // In word mode (the default) we have data bus samples, plus optionally
      // rnw, sync, rdy, phy2 and rst.

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

   // Flush the lookhead decoder so it
   if (idx_sync < 0) {
      for (int i = 0; i < DEPTH - 1; i++) {
         lookahead_decode_cycle_without_sync(0xEA, 1, 1);
      }
   }

}


// ====================================================================
// Main program entry point
// ====================================================================

int main(int argc, char *argv[]) {
   arguments.idx_data         =  0;
   arguments.idx_rnw          =  8;
   arguments.idx_sync         =  9;
   arguments.idx_rdy          = 10;
   arguments.idx_phi2         = 11;
   arguments.idx_rst          = 14;
   arguments.vec_rst          = 0xA9D9CD; // These are the defaults for the beeb
   arguments.machine          = MACHINE_DEFAULT;

   arguments.show_address     = 1;
   arguments.show_hex         = 0;
   arguments.show_instruction = 1;
   arguments.show_state       = 0;
   arguments.show_bbcfwa      = 0;
   arguments.show_cycles      = 0;

   arguments.bbctube          = 0;
   arguments.emulate          = 0;
   arguments.c02              = 0;
   arguments.rockwell         = 0;
   arguments.undocumented     = 0;
   arguments.byte             = 0;
   arguments.debug            = 0;
   arguments.profile          = 0;
   arguments.trigger_start    = -1;
   arguments.trigger_stop     = -1;
   arguments.trigger_skipint  = 0;
   arguments.filename         = NULL;

   argp_parse(&argp, argc, argv, 0, 0, &arguments);

   arguments.show_something = arguments.show_address | arguments.show_hex | arguments.show_instruction | arguments.show_state | arguments.show_bbcfwa | arguments.show_cycles;

   // Normally the data file should be 16 bit samples. In byte mode
   // the data file is 8 bit samples, and all the control signals are
   // assumed to be don't care.
   if (arguments.byte) {
      arguments.idx_rnw  = -1;
      arguments.idx_sync = -1;
      arguments.idx_rdy  = -1;
      arguments.idx_phi2 = -1;
      arguments.idx_rst  = -1;
   }

   if (arguments.emulate || arguments.bbctube || arguments.show_state || arguments.show_bbcfwa || arguments.idx_sync < 0 || arguments.idx_rnw < 0) {
      do_emulate = 1;
   }

   if (arguments.profile) {
      profiler_init();
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
   em_init(arguments.c02, arguments.rockwell, arguments.undocumented, arguments.bbctube);
   decode(stream);
   fclose(stream);

   if (arguments.profile) {
      profiler_done();
   }

   return 0;
}
