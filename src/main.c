#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <argp.h>
#include <string.h>
#include <math.h>

#include "defs.h"
#include "em_6502.h"
#include "em_65816.h"
#include "profiler.h"

int sample_count = 0;

#define BUFSIZE 8192

uint8_t buffer8[BUFSIZE];

uint16_t buffer[BUFSIZE];

const char *machine_names[] = {
   "default",
   "master",
   "elk",
   0
};

// escaping is to avoid unwanted trigraphs
const char default_fwa[] = "\?\?-\?\?:\?\?\?\?\?\?\?\?:\?\?:\?\? = \?\?\?\?\?\?\?\?\?\?\?\?\?\?\?";

#define OFFSET_SIGN      0
#define OFFSET_EXP       3
#define OFFSET_MANTISSA  6
#define OFFSET_ROUND    15
#define OFFSET_OVERFLOW 18
#define OFFSET_VALUE    23

static char fwabuf[80];
static char disbuf[256];

static cpu_emulator_t *em;

static int c816;

// This is a global, so it's visible to the emulator functions
arguments_t arguments;

// This is a global, so it's visible to the emulator functions
int triggered = 0;

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
   { "c02",          'c',        0,                   0, "Enable 65C02 mode."},
   { "c816",         '8',        0,                   0, "Enable 65C816 mode."},
   { "rockwell",     'r',        0,                   0, "Enable additional rockwell instructions."},
   { "undocumented", 'u',        0,                   0, "Enable undocumented 6502 opcodes (currently incomplete)"},
   { "byte",         'b',        0,                   0, "Byte samples"},
   { "debug",        'd',  "LEVEL",                   0, "Sets debug level (bitmask)"},
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
   { "vda",           9,  "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for vda, blank if unconnected"},
   { "vpa",          10,  "BITNUM", OPTION_ARG_OPTIONAL, "The bit number for vpa, blank if unconnected"},
   { "emul",         11,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the E flag in 65816 mode"},
   { "sp",           12,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Stack Pointer register (65816)"},
   { "pb",           13,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Program Bank register (65816)"},
   { "db",           14,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Data Bank register (65816)"},
   { "dp",           15,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Direct Page register (65816)"},
   { "ms",           16,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the M flag (65816)"},
   { "xs",           17,     "HEX", OPTION_ARG_OPTIONAL, "Initial value of the X flag (65816)"},
   { "skip",         18,     "HEX", OPTION_ARG_OPTIONAL, "Skip n samples"},
   { 0 }
};

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
   int i;
   arguments_t *arguments = state->input;

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
   case   9:
      if (arg && strlen(arg) > 0) {
         arguments->idx_vda = atoi(arg);
      } else {
         arguments->idx_vda = -1;
      }
      break;
   case  10:
      if (arg && strlen(arg) > 0) {
         arguments->idx_vpa = atoi(arg);
      } else {
         arguments->idx_vpa = -1;
      }
      break;
   case  11:
      if (arg && strlen(arg) > 0) {
         arguments->e_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->e_flag = -1;
      }
      break;
   case  12:
      if (arg && strlen(arg) > 0) {
         arguments->sp_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->sp_reg = -1;
      }
      break;
   case  13:
      if (arg && strlen(arg) > 0) {
         arguments->pb_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->pb_reg = -1;
      }
      break;
   case  14:
      if (arg && strlen(arg) > 0) {
         arguments->db_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->db_reg = -1;
      }
      break;
   case  15:
      if (arg && strlen(arg) > 0) {
         arguments->dp_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->dp_reg = -1;
      }
      break;
   case  16:
      if (arg && strlen(arg) > 0) {
         arguments->ms_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->ms_flag = -1;
      }
      break;
   case  17:
      if (arg && strlen(arg) > 0) {
         arguments->xs_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->xs_flag = -1;
      }
      break;
   case  18:
      if (arg && strlen(arg) > 0) {
         arguments->skip = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->skip = 0;
      }
      break;
   case 'c':
      arguments->cpu_type = CPU_65C02;
      break;
   case '8':
      arguments->cpu_type = CPU_65C816;
      break;
   case 'r':
      arguments->cpu_type = CPU_65C02_ROCKWELL;
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


static void dump_samples(sample_t *sample_q, int n) {
      for (int i = 0; i < n; i++) {
         sample_t *sample = sample_q + i;
         printf("%d %02x ", i, sample->data);
         switch(sample->type) {
         case INTERNAL:
            putchar('I');
            break;
         case PROGRAM:
            putchar('P');
            break;
         case DATA:
            putchar('D');
            break;
         case OPCODE:
            putchar('O');
            break;
         case LAST:
            putchar('L');
            break;
         default:
            putchar('?');
            break;
         }
         putchar(' ');
         putchar(sample->rnw >= 0 ? '0' + sample->rnw : '?');
         putchar(' ');
         putchar(sample->rst >= 0 ? '0' + sample->rst : '?');
         putchar('\n');
      }

}

void write_hex1(char *buffer, int value) {
   *buffer = value + (value < 10 ? '0' : 'A' - 10);
}

void write_hex2(char *buffer, int value) {
   write_hex1(buffer++, (value >> 4) & 15);
   write_hex1(buffer++, (value >> 0) & 15);
}

void write_hex4(char *buffer, int value) {
   write_hex1(buffer++, (value >> 12) & 15);
   write_hex1(buffer++, (value >> 8) & 15);
   write_hex1(buffer++, (value >> 4) & 15);
   write_hex1(buffer++, (value >> 0) & 15);
}

void write_hex6(char *buffer, int value) {
   write_hex1(buffer++, (value >> 20) & 15);
   write_hex1(buffer++, (value >> 16) & 15);
   write_hex1(buffer++, (value >> 12) & 15);
   write_hex1(buffer++, (value >> 8) & 15);
   write_hex1(buffer++, (value >> 4) & 15);
   write_hex1(buffer++, (value >> 0) & 15);
}

int write_s(char *buffer, const char *s) {
   int i = 0;
   while (*s) {
      *buffer++ = *s++;
      i++;
   }
   return i;
}

static char *get_fwa(int a_sign, int a_exp, int a_mantissa, int a_round, int a_overflow) {
   strcpy(fwabuf, default_fwa);
   int sign     = em->read_memory(a_sign);
   int exp      = em->read_memory(a_exp);
   int man1     = em->read_memory(a_mantissa);
   int man2     = em->read_memory(a_mantissa + 1);
   int man3     = em->read_memory(a_mantissa + 2);
   int man4     = em->read_memory(a_mantissa + 3);
   int round    = em->read_memory(a_round);
   int overflow = a_overflow >= 0 ? em->read_memory(a_overflow) : -1;
   if (sign >= 0) {
      write_hex2(fwabuf + OFFSET_SIGN, sign);
   }
   if (exp >= 0) {
      write_hex2(fwabuf + OFFSET_EXP, exp);
   }
   if (man1 >= 0) {
      write_hex2(fwabuf + OFFSET_MANTISSA + 0, man1);
   }
   if (man2 >= 0) {
      write_hex2(fwabuf + OFFSET_MANTISSA + 2, man2);
   }
   if (man3 >= 0) {
      write_hex2(fwabuf + OFFSET_MANTISSA + 4, man3);
   }
   if (man4 >= 0) {
      write_hex2(fwabuf + OFFSET_MANTISSA + 6, man4);
   }
   if (round >= 0) {
      write_hex2(fwabuf + OFFSET_ROUND, round);
   }
   if (overflow >= 0) {
      write_hex2(fwabuf + OFFSET_OVERFLOW, overflow);
   }
   if (sign >= 0 && exp >= 0 && man1 >= 0 && man2 >= 0 && man3 >= 0 && man4 >= 0 && round >= 0) {

      // Real numbers are held in binary floating point format. In the
      // default (40-bit) mode the mantissa is held as a 4 byte binary
      // fraction in sign and magnitude format. Bit 7 of the MSB of
      // the mantissa is the sign bit. When working out the value of
      // the mantissa, this bit is assumed to be 1 (a decimal value of
      // 0.5). The exponent is held as a single byte in 'excess 127'
      // format. In other words, if the actual exponent is zero, the
      // value stored in the exponent byte is 127.

      // Build up a 32 bit mantissa
      uint64_t mantissa = man1;
      mantissa = (mantissa << 8) + man2;
      mantissa = (mantissa << 8) + man3;
      mantissa = (mantissa << 8) + man4;

      // Extend this to 40 bits with the rounding byte
      mantissa = (mantissa << 8) + round;

      // Combine with the exponent
      double value = ((double) mantissa) * pow(2.0, exp - 128 - 40);
      // Take account of the sign
      if (sign & 128) {
         value = -value;
      }
      // Print it to the fwabuf
      sprintf(fwabuf + OFFSET_VALUE, "%-+15.8E", value);
   }
   return fwabuf;
}


static int analyze_instruction(sample_t *sample_q, int num_samples, int rst_seen) {
   static int total_cycles = 0;
   static int interrupt_depth = 0;
   static int skipping_interrupted = 0;

   int intr_seen = em->match_interrupt(sample_q, num_samples);

   int num_cycles;

   if (rst_seen > 0) {
      num_cycles = rst_seen;
   } else {
      num_cycles = em->count_cycles(sample_q, intr_seen);
   }

   // Deal with partial final instruction
   if (num_samples <= num_cycles || num_cycles == 0) {
      return num_samples;
   }

   if (triggered && arguments.debug & 1) {
      dump_samples(sample_q, num_cycles);
   }

   instruction_t instruction;

   int oldpc = em->get_PC();
   int oldpb = em->get_PB();

   if (rst_seen) {
      // Handle a reset
      em->reset(sample_q, num_cycles, &instruction);
   } else if (intr_seen) {
      // Handle an interrupt
      em->interrupt(sample_q, num_cycles, &instruction);
   } else {
      // Handle a normal instruction
      em->emulate(sample_q, num_cycles, &instruction);
   }

   // Sanity check the pc prediction has not gone awry
   // (e.g. in JSR the emulation can use the stacked PC)

   int opcode = instruction.opcode;
   int pb = instruction.pb;
   int pc = instruction.pc;

   if (c816) {
      if (pb >= 0) {
         if (oldpb >= 0 && oldpb != pb) {
            printf("pb: prediction failed at %02X old pb was %02X\n", pb, oldpb);
         }
      }
   }

   if (pc >= 0) {
      if (oldpc >= 0 && oldpc != pc) {
         printf("pc: prediction failed at %04X old pc was %04X\n", pc, oldpc);
      }
   }

   if (pc >= 0 && pc == arguments.trigger_start) {
      triggered = 1;
      printf("start trigger hit at cycle %d\n", total_cycles);
   } else if (pc >= 0 && pc == arguments.trigger_stop) {
      triggered = 0;
      printf("stop trigger hit at cycle %d\n", total_cycles);
   }

   // Exclude interrupts from profiling
   if (arguments.trigger_skipint && pc >= 0) {
      if (interrupt_depth == 0) {
         skipping_interrupted = 0;
      }
      if (intr_seen) {
         interrupt_depth++;
         skipping_interrupted = 1;
      } else if (interrupt_depth > 0 && opcode == 0x40) {
         interrupt_depth--;
      }
   }

   if (arguments.profile && triggered && !skipping_interrupted && !intr_seen) {
      // TODO: refactor profiler to take instruction_t *
      profiler_profile_instruction(instruction.pc, instruction.opcode, instruction.op1, instruction.op2, num_cycles);
   }

   int fail = em->get_and_clear_fail();

   // Try to minimise the calls to printf as these are quite expensive

   char *bp = disbuf;

   if ((fail | arguments.show_something) && triggered && !skipping_interrupted) {
      int numchars = 0;
      // Show address
      if (fail || arguments.show_address) {
         if (c816) {
            if (pb < 0) {
               *bp++ = '?';
               *bp++ = '?';
            } else {
               write_hex2(bp, pb);
               bp += 2;
            }
         }
         if (pc < 0) {
            *bp++ = '?';
            *bp++ = '?';
            *bp++ = '?';
            *bp++ = '?';
         } else {
            write_hex4(bp, pc);
            bp += 4;
         }
         *bp++ = ' ';
         *bp++ = ':';
         *bp++ = ' ';
      }
      // Show hex bytes
      if (fail || arguments.show_hex) {
         for (int i = 0; i < (c816 ? 4 : 3); i++) {
            if (rst_seen || intr_seen || i > instruction.opcount) {
               *bp++ = ' ';
               *bp++ = ' ';
            } else {
               switch (i) {
               case 0: write_hex2(bp, opcode         ); break;
               case 1: write_hex2(bp, instruction.op1); break;
               case 2: write_hex2(bp, instruction.op2); break;
               case 3: write_hex2(bp, instruction.op3); break;
               }
               bp += 2;
            }
            *bp++ = ' ';
         }
         *bp++ = ':';
         *bp++ = ' ';
      }

      // Show instruction disassembly
      if (fail || arguments.show_something) {
         if (rst_seen) {
            numchars = write_s(bp, "RESET !!");
         } else if (intr_seen) {
            numchars = write_s(bp, "INTERRUPT !!");
         } else {
            numchars = em->disassemble(bp, &instruction);
         }
         bp += numchars;
      }

      // Pad if there is more to come
      if (fail || arguments.show_cycles || arguments.show_state || arguments.show_bbcfwa) {
         // Pad opcode to 14 characters, to match python
         while (numchars++ < 14) {
            *bp++ = ' ';
         }
      }
      // Show cycles (don't include with fail as it is inconsistent depending on whether rdy is present)
      if (arguments.show_cycles) {
         *bp++ = ' ';
         *bp++ = ':';
         *bp++ = ' ';
         // No instruction is more then 8 cycles
         write_hex1(bp++, num_cycles);
      }
      // Show register state
      if (fail || arguments.show_state) {
         *bp++ = ' ';
         *bp++ = ':';
         *bp++ = ' ';
         bp = em->get_state(bp);
      }
      // Show BBC floating point work area FWA, FWB
      if (arguments.show_bbcfwa) {
         bp += sprintf(bp, " : FWA %s", get_fwa(0x2e, 0x30, 0x31, 0x35, 0x2f));
         bp += sprintf(bp, " : FWB %s", get_fwa(0x3b, 0x3c, 0x3d, 0x41,   -1));
      }
      // Show any errors
      if (fail) {
         bp += write_s(bp, " prediction failed");
      }
      // End the line
      *bp++ = 0;
      puts(disbuf);

   }

   total_cycles += num_cycles;
   return num_cycles;
}

// ====================================================================
// Generic instruction decoder
// ====================================================================

// This stage is mostly about cleaning coming out of reset in all cases
//
//        Rst Sync
//
// Case 1:  ?   ?  : search for heuristic at n, n+1, n+2 - consume n+2 cycles
// Case 2:  ?  01  : search for heuristic at 5, 6, 7 - consume 7 ctcles
// Case 3: 01   ?  : dead reconning; 8 or 9 depending on the cpu type
// Case 4: 01  01  : mark first instruction after rst stable
//

int decode_instruction(sample_t *sample_q, int num_samples) {
   static int rst_seen = 0;

   // Skip any samples where RST is asserted (active low)
   if (sample_q[0].rst == 0) {
      rst_seen = 1;
      return 1;
   }

   // If the first sample is not an SYNC, then drop the sample
   if (sample_q->type != OPCODE && sample_q->type != UNKNOWN) {
      return 1;
   }

   // Flag to indicate the sample type is missing (sync/vda/vpa unconnected)
   int notype = sample_q[0].type == UNKNOWN;

   if (arguments.idx_rst < 0) {
      // We use a heuristic, based on what we expect to see on the data
      // bus in cycles 5, 6 and 7, i.e. RSTVECL, RSTVECH, RSTOPCODE
      int veclo  = (arguments.vec_rst      ) & 0xff;
      int vechi  = (arguments.vec_rst >>  8) & 0xff;
      int opcode = (arguments.vec_rst >> 16) & 0xff;
      if (notype) {
         // No Sync, so search for heurisic anywhere in the sample queue
         for (int i = 0; i <= num_samples - 3; i++) {
            if (sample_q[i].data == veclo && sample_q[i + 1].data == vechi && (!opcode || sample_q[i + 2].data == opcode)) {
               rst_seen = i + 2;
               break;
            }
         }
      } else {
         // Sync, so check heurisic at a specific offset in the sample queue
         if (sample_q[5].data == veclo && sample_q[6].data == vechi && (!opcode || sample_q[7].data == opcode) && sample_q[7].type == OPCODE) {
            rst_seen = 7;
         }
      }
   } else if (rst_seen) {
      // First, make sure rst is stable
      for (int i = 1; i < num_samples; i++) {
         if (sample_q[i].rst == 0) {
            return i + 1;
         }
      }
      if (notype) {
         // Do this by dead reconning
         rst_seen = (arguments.cpu_type == CPU_65C02 || arguments.cpu_type == CPU_65C02_ROCKWELL) ? 8 : 9;
         // We could also check the vector
      } else {
         if (sample_q[7].type == OPCODE) {
            rst_seen = 7;
         } else {
            printf("Instruction after rst /= 7 cycles\n");
            rst_seen = 0;
         }
      }
   }

   // Decode the instruction
   int num_cycles = analyze_instruction(sample_q, num_samples, rst_seen);

   // And reset rst_seen for the next reset
   if (rst_seen) {
      rst_seen = 0;
   }

   return num_cycles;
}

// ====================================================================
// Queue a small number of samples so the decoders can lookahead
// ====================================================================

void queue_sample(sample_t *sample) {
   static sample_t sample_q[DEPTH];
   static int index = 0;

   sample_q[index++] = *sample;

   if (sample->type == LAST) {
      // To prevent edge condition, don't advertise the LAST marker
      index--;
      // Drain the queue when the LAST marker is seen
      while (index > 1) {
         int consumed = decode_instruction(sample_q, index);
         for (int i = 0; i < DEPTH - consumed; i++) {
            sample_q[i] = sample_q[i + consumed];
         }
         index -= consumed;
      }
   } else {
      // Else queue the samples
      // If the queue is full, then pass on to the decoder
      if (index == DEPTH) {
         int consumed = decode_instruction(sample_q, index);
         for (int i = 0; i < DEPTH - consumed; i++) {
            sample_q[i] = sample_q[i + consumed];
         }
         index -= consumed;
      }
   }
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
   int idx_vda   = arguments.idx_vda;
   int idx_vpa   = arguments.idx_vpa;

   // Default Pin values
   int bus_data  =  0;
   int pin_rnw   =  1;
   int pin_sync  =  0;
   int pin_rdy   =  1;
   int pin_phi2  =  0;
   int pin_rst   =  1;
   int pin_vda   =  0;
   int pin_vpa   =  0;

   int num;

   // The previous sample of the 16-bit capture (async sampling only)
   uint16_t sample       = -1;
   uint16_t last_sample  = -1;
   uint16_t last2_sample = -1;

   // The previous sample of phi2 (async sampling only)
   int last_phi2 = -1;

   sample_t s;

   // Skip the start of the file, if required
   if (arguments.skip) {
      fseek(stream, arguments.skip * (arguments.byte ? 1 : 2), SEEK_SET);
   }

   if (arguments.byte) {
      s.rnw = -1;
      s.rst = -1;
      s.type = UNKNOWN;

      // In byte mode we have only data bus samples, nothing else so we must
      // use the sync-less decoder. The values of pin_rnw and pin_rst are set
      // to 1, but the decoder should never actually use them.

      while ((num = fread(buffer8, sizeof(uint8_t), BUFSIZE, stream)) > 0) {
         uint8_t *sampleptr = &buffer8[0];
         while (num-- > 0) {
            s.data = *sampleptr++;
            queue_sample(&s);
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
            //if (arguments.debug & 4) {
            //   printf("%d %02x %x %x %x %x\n", sample_count, sample&255, (sample >> 8)&1,  (sample >> 9)&1,  (sample >> 10)&1,  (sample >> 11)&1  );
            //}
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
               if (c816) {
                  if (idx_vda >= 0) {
                     pin_vda = (sample >> idx_vda) & 1;
                  }
                  if (idx_vpa >= 0) {
                     pin_vpa = (sample >> idx_vpa) & 1;
                  }
               } else {
                  if (idx_sync >= 0) {
                     pin_sync = (sample >> idx_sync) & 1;
                  }
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
                  if (!c816) {
                     if (idx_rnw >= 0) {
                        pin_rnw = (sample >> idx_rnw ) & 1;
                     }
                     if (c816) {
                        if (idx_vda >= 0) {
                           pin_vda = (sample >> idx_vda) & 1;
                        }
                        if (idx_vpa >= 0) {
                           pin_vpa = (sample >> idx_vpa) & 1;
                        }
                     } else {
                        if (idx_sync >= 0) {
                           pin_sync = (sample >> idx_sync) & 1;
                        }
                     }
                     if (idx_rst >= 0) {
                        pin_rst = (sample >> idx_rst) & 1;
                     }
                  }
                  // continue for more samples
                  continue;
               } else {
                  if (idx_rdy >= 0) {
                     pin_rdy = (last_sample >> idx_rdy) & 1;
                  }
                  if (c816) {
                     if (idx_rnw >= 0) {
                        pin_rnw = (last_sample >> idx_rnw ) & 1;
                     }
                     if (idx_vda >= 0) {
                        pin_vda = (last_sample >> idx_vda) & 1;
                     }
                     if (idx_vpa >= 0) {
                        pin_vpa = (last_sample >> idx_vpa) & 1;
                     }
                     if (idx_rst >= 0) {
                        pin_rst = (last_sample >> idx_rst) & 1;
                     }
                     // sample data just before falling edge of Phi2
                     bus_data = last_sample & 255;
                  } else if (arguments.machine == MACHINE_ELK) {
                     // sample data just before falling edge of Phi2
                     bus_data = last_sample & 255;
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

            // Build the sample
            if (c816) {
               if (idx_vda < 0 || idx_vpa < 0) {
                  s.type = UNKNOWN;
               } else {
                  s.type = pin_vpa ? (pin_vda ? OPCODE : PROGRAM) : (pin_vda ? DATA : INTERNAL);
               }
            } else {
               if (idx_sync < 0) {
                  s.type = UNKNOWN;
               } else {
                  s.type = pin_sync ? OPCODE : DATA;
               }
            }
            s.data = bus_data;
            if (idx_rnw < 0) {
               s.rnw = -1;
            } else {
               s.rnw = pin_rnw;
            }
            if (idx_rst < 0) {
               s.rst = -1;
            } else {
               s.rst = pin_rst;
            }
            queue_sample(&s);
         }
      }
   }

   // Flush the sample queue
   s.type = LAST;
   queue_sample(&s);
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
   arguments.idx_vpa          =  9;
   arguments.idx_vda          = 11;
   arguments.vec_rst          = 0xA9D9CD; // These are the defaults for the beeb
   arguments.machine          = MACHINE_DEFAULT;

   arguments.show_address     = 1;
   arguments.show_hex         = 0;
   arguments.show_instruction = 1;
   arguments.show_state       = 0;
   arguments.show_bbcfwa      = 0;
   arguments.show_cycles      = 0;

   arguments.bbctube          = 0;
   arguments.cpu_type         = CPU_6502;
   arguments.undocumented     = 0;
   arguments.e_flag           = -1;
   arguments.sp_reg           = -1;
   arguments.pb_reg           = -1;
   arguments.db_reg           = -1;
   arguments.dp_reg           = -1;
   arguments.ms_flag          = -1;
   arguments.xs_flag          = -1;
   arguments.byte             = 0;
   arguments.debug            = 0;
   arguments.skip             = 0;
   arguments.profile          = 0;
   arguments.trigger_start    = -1;
   arguments.trigger_stop     = -1;
   arguments.trigger_skipint  = 0;
   arguments.filename         = NULL;

   argp_parse(&argp, argc, argv, 0, 0, &arguments);

   if (arguments.trigger_start < 0) {
      triggered = 1;
   }

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
      arguments.idx_vpa  = -1;
      arguments.idx_vda  = -1;
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

   if (arguments.cpu_type == CPU_65C816) {
      c816 = 1;
      em = &em_65816;
   } else {
      c816 = 0;
      em = &em_6502;
   }
   em->init(&arguments);

   decode(stream);
   fclose(stream);

   if (arguments.profile) {
      profiler_done();
   }

   return 0;
}
