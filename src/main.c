#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <argp.h>
#include <string.h>
#include <math.h>

#include "defs.h"
#include "em_6502.h"
#include "em_65816.h"
#include "em_6800.h"
#include "memory.h"
#include "profiler.h"
#include "symbols.h"

// Small skew buffer to allow the data bus samples to be taken early or late

#define SKEW_BUFFER_SIZE  32 // Must be a power of 2

#define MAX_SKEW_VALUE ((SKEW_BUFFER_SIZE / 2) - 1)

// Value use to indicate a pin (or register) has not been assigned by
// the user, so should take the default value.
#define UNSPECIFIED -2

// Value used to indicate a pin (or register) is undefined. For a pin,
// this means unconnected. For a register this means it will default
// to a value of undefined (?).
#define UNDEFINED -1

#define BUFSIZE 8192

uint8_t buffer8[BUFSIZE];

uint16_t buffer[BUFSIZE];

const char *machine_names[] = {
   "default",
   "beeb",
   "master",
   "elk",
   "atom",
   "mek6800d2",
   "blitter",
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
static int arlet;

// This is a global, so it's visible to the emulator functions
arguments_t arguments;

// This is a global, so it's visible to the emulator functions
int triggered = 0;

// indicate state prediction failed
int failflag = 0;

// ====================================================================
// Argp processing
// ====================================================================

const char *argp_program_version = "decode6502 0.1";

const char *argp_program_bug_address = "<dave@hoglet.com>";

static char doc[] = "\n\
Decoder for 6502/65C02/65C816 logic analyzer capture files.\n\
\n\
FILENAME must be a binary capture file containing:\n\
- 16 bit samples (of the data bus and control signals), or\n\
-  8-bit samples (of the data bus), if the --byte option is present.\n\
\n\
If FILENAME is omitted, stdin is read instead.\n\
\n\
The default sample bit assignments for the 6502/65C02 signals are:\n\
 - data: bit  0 (assumes 8 consecutive bits)\n\
 -  rnw: bit  8\n\
 - sync: bit  9\n\
 -  rdy: bit 10\n\
 -  rst: bit 14\n\
 - phi2: bit 15\n\
\n\
The default sample bit assignments for the 65C816 signals are:\n\
 - data: bit  0 (assumes 8 consecutive bits)\n\
 -  rnw: bit  8\n\
 -  vpa: bit  9\n\
 -  rdy: bit 10\n\
 -  vda: bit 11\n\
 -    e: bit 12\n\
 -  rst: bit 14\n\
 - phi2: bit 15\n\
\n\
To specify that an input is unconnected, include the option with an empty\n\
BITNUM. e.g. --sync=\n\
\n\
If phi2 is not connected the capture file should contain one sample per\n\
falling edge of phi2.\n\
\n\
If rdy is not connected a value of '1' is assumed.\n\
\n\
If sync (or vda/vpa) is not connected a heuristic based decoder is used.\n\
This works well, but can take several instructions to lock onto the\n\
instruction stream. Use of sync (or vda/vpa) is recommended.\n\
\n\
If RST is not connected, an alternative is to specify the reset vector:\n\
 - D9CD (D9 is the high byte, CD is the low byte)\n\
 - A9D9CD (optionally, also specify the first opcode, LDA # in this case)\n\
\n\
If --debug=1 is specified, each instruction is preceeded by it\'s sample values.\n\
\n\
The --mem= option controls the memory access logging and modelling. The value\n\
is three hex nibbles: WRM, where W controls write logging, R controls read\n\
logging, and M controls modelling.\n\
Each of the three nibbles has the same semantics:\n\
 - bit 3 applies to stack accesses\n\
 - bit 2 applies to data accesses\n\
 - bit 1 applies to pointer accesses\n\
 - bit 0 applies to instruction accesses\n\
Examples:\n\
 --mem=00F models (and verifies) all accesses, but with minimal extra logging\n\
 --mem=F0F would additional log all writes\n\
\n";

static char args_doc[] = "[FILENAME]";

enum {
   GROUP_GENERAL = 1,
   GROUP_OUTPUT  = 2,
   GROUP_SIGDEFS = 3,
   GROUP_6502    = 4,
   GROUP_65816   = 5
};


enum {
   KEY_ADDR = 'a',
   KEY_BYTE = 'b',
   KEY_CPU = 'c',
   KEY_DEBUG = 'd',
   KEY_BBCFWA = 'f',
   KEY_HEX = 'h',
   KEY_INSTR = 'i',
   KEY_MACHINE = 'm',
   KEY_PROFILE = 'p',
   KEY_QUIET = 'q',
   KEY_STATE = 's',
   KEY_TRIGGER = 't',
   KEY_UNDOC = 'u',
   KEY_CYCLES = 'y',
   KEY_SAMPLES = 'Y',
   KEY_VECRST = 1,
   KEY_BBCTUBE,
   KEY_MEM,
   KEY_SP,
   KEY_SKIP,
   KEY_SKEW,
   KEY_SKEW_RD,
   KEY_SKEW_WR,
   KEY_LABELS,
   KEY_DATA,
   KEY_RNW,
   KEY_RDY,
   KEY_PHI1,
   KEY_PHI2,
   KEY_USER,
   KEY_RST,
   KEY_SYNC,
   KEY_VDA,
   KEY_VPA,
   KEY_E,
   KEY_PB,
   KEY_DB,
   KEY_DP,
   KEY_EMUL,
   KEY_MS,
   KEY_XS,
   KEY_SHOWROM = 'r'
};


typedef struct {
   char *cpu_name;
   cpu_t cpu_type;
} cpu_name_t;

static cpu_name_t cpu_names[] = {
   // 6502
   {"6502",       CPU_6502},
   {"R6502",      CPU_6502},
   {"SY6502",     CPU_6502},
   {"NMOS",       CPU_6502},
   {"02",         CPU_6502},
   // 6502_ARLET
   {"ARLET",      CPU_6502_ARLET},
   // 65C02
   {"65C02",      CPU_65C02},
   {"CMOS",       CPU_65C02},
   {"C02",        CPU_65C02},
   // 65C02_ROCKWELL
   {"R65C02",     CPU_65C02_ROCKWELL},
   {"ROCKWELL",   CPU_65C02_ROCKWELL},
   // 65C02_WDC
   {"WD65C02",    CPU_65C02_WDC},
   {"W65C02",     CPU_65C02_WDC},
   {"WDC",        CPU_65C02_WDC},
   // 65C02_ARLET
   {"ARLETC02",   CPU_65C02_ARLET},
   // 65C02_ALAND
   {"ALANDC02",   CPU_65C02_ALAND},
   // 65C816
   {"65816",      CPU_65C816},
   {"65C816",     CPU_65C816},
   {"W65816",     CPU_65C816},
   {"W65C816",    CPU_65C816},
   {"816",        CPU_65C816},
   {"C816",       CPU_65C816},
   // 6800
   {"6800",       CPU_6800},
   {"M6800",      CPU_6800},
   {"MC6800",     CPU_6800},
   {"6802",       CPU_6800},
   {"M6802",      CPU_6800},
   {"MC6802",     CPU_6800},

   // Terminator
   {NULL, 0}
};

static int cpu_rst_delay[] = {
   9, // CPU_UNKNOWN
   9, // CPU_6502
   9, // CPU_6502_ARLET
   8, // CPU_65C02
   8, // CPU_65C02_ROCKWELL
   8, // CPU_65C02_WDC
   9, // CPU_65C02_ARLET
   9, // CPU_65C02_ALAND
   9, // CPU_65C816
   3, // CPU_6800
};

static struct argp_option options[] = {
   { 0, 0, 0, 0, "General options:", GROUP_GENERAL},

   { "vecrst",      KEY_VECRST,    "HEX",  OPTION_ARG_OPTIONAL, "Reset vector, optionally preceeded by the first opcode (e.g. A9D9CD)",
                                                                                                                     GROUP_GENERAL},
   { "cpu",            KEY_CPU,     "CPU",                   0, "Sets CPU type (6502, 65c02, r65c02, 65c816)",       GROUP_GENERAL},
   { "machine",    KEY_MACHINE, "MACHINE",                   0, "Enable machine (beeb,elk,master) defaults",         GROUP_GENERAL},
   { "byte",          KEY_BYTE,         0,                   0, "Enable byte-wide sample mode",                      GROUP_GENERAL},
   { "debug",        KEY_DEBUG,   "LEVEL",                   0, "Sets the debug level (0 or 1)",                     GROUP_GENERAL},
   { "profile",    KEY_PROFILE,  "PARAMS", OPTION_ARG_OPTIONAL, "Profile code execution",                            GROUP_GENERAL},
   { "trigger",    KEY_TRIGGER, "ADDRESS",                   0, "Trigger on address",                                GROUP_GENERAL},
   { "bbctube",    KEY_BBCTUBE,         0,                   0, "BBC tube protocol decoding",                        GROUP_GENERAL},
   { "mem",            KEY_MEM,     "HEX", OPTION_ARG_OPTIONAL, "Memory modelling (see above)",                      GROUP_GENERAL},
   { "skip",          KEY_SKIP,     "HEX", OPTION_ARG_OPTIONAL, "Skip the first n samples",                          GROUP_GENERAL},
   { "skew",          KEY_SKEW,    "SKEW", OPTION_ARG_OPTIONAL, "Skew the data bus by +/- n samples",                GROUP_GENERAL},
   { "skew_rd",    KEY_SKEW_RD,    "SKEW", OPTION_ARG_OPTIONAL, "Skew the data bus by +/- n samples for read data",  GROUP_GENERAL},
   { "skew_wr",    KEY_SKEW_WR,    "SKEW", OPTION_ARG_OPTIONAL, "Skew the data bus by +/- n samples for write data", GROUP_GENERAL},
   { "labels",      KEY_LABELS,   "FILE",                    0, "Swift format label/symbols file e.g. from beebasm", GROUP_GENERAL},

   { 0, 0, 0, 0, "Output options:", GROUP_OUTPUT},

   { "quiet",        KEY_QUIET,         0,                   0, "Set all the output options to off",                 GROUP_OUTPUT},
   { "address",       KEY_ADDR,         0,                   0, "Show address of instruction",                       GROUP_OUTPUT},
   { "hex",            KEY_HEX,         0,                   0, "Show hex bytes of instruction",                     GROUP_OUTPUT},
   { "instruction",  KEY_INSTR,         0,                   0, "Show instruction disassembly",                      GROUP_OUTPUT},
   { "state",        KEY_STATE,         0,                   0, "Show register/flag state",                          GROUP_OUTPUT},
   { "cycles",      KEY_CYCLES,         0,                   0, "Show number of bus cycles",                         GROUP_OUTPUT},
   { "samplenum",  KEY_SAMPLES,         0,                   0, "Show bus cycle numbers",                            GROUP_OUTPUT},
   { "bbcfwa",      KEY_BBCFWA,         0,                   0, "Show BBC floating-point work areas",                GROUP_OUTPUT},
   { "showromno",   KEY_SHOWROM,        0,                   0, "Show BBC rom no for address 8000..BFFF",            GROUP_OUTPUT},

   { 0, 0, 0, 0, "Signal defintion options:", GROUP_SIGDEFS},

   { "data",          KEY_DATA, "BITNUM",                   0, "Bit number for data (default  0)",                   GROUP_SIGDEFS},
   { "rnw",            KEY_RNW, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for rnw  (default  8)",                   GROUP_SIGDEFS},
   { "rdy",            KEY_RDY, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for rdy  (default 10)",                   GROUP_SIGDEFS},
   { "phi1",          KEY_PHI1, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for phi1 (default -1)",                   GROUP_SIGDEFS},
   { "phi2",          KEY_PHI2, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for phi2 (default 15)",                   GROUP_SIGDEFS},
   { "user",          KEY_USER, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for user (default -1)",                   GROUP_SIGDEFS},
   { "rst",            KEY_RST, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for rst  (default 14)",                   GROUP_SIGDEFS},
   { "sync",          KEY_SYNC, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for sync (default  9) (6502/65C02)",      GROUP_SIGDEFS},
   { "vpa",            KEY_VPA, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for vpa  (default  9) (65C816)",          GROUP_SIGDEFS},
   { "vda",            KEY_VDA, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for vda  (default 11) (65C816)",          GROUP_SIGDEFS},
   { "e",                KEY_E, "BITNUM", OPTION_ARG_OPTIONAL, "Bit number for e    (default 12) (65C816)",          GROUP_SIGDEFS},
   { 0, 0, 0, 0, "Additional 6502/65C02 options:", GROUP_6502},

   { "undocumented", KEY_UNDOC,        0,                   0, "Enable undocumented opcodes",                        GROUP_6502},
   { "sp",              KEY_SP,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Stack Pointer register",       GROUP_6502},

   { 0, 0, 0, 0, "Additional 65C816 options:", GROUP_65816},

   { "pb",              KEY_PB,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Program Bank register",         GROUP_65816},
   { "db",              KEY_DB,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Data Bank register",            GROUP_65816},
   { "dp",              KEY_DP,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Direct Page register",          GROUP_65816},
   { "emul",          KEY_EMUL,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the E flag",                        GROUP_65816},
   { "ms",              KEY_MS,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the M flag",                        GROUP_65816},
   { "xs",              KEY_XS,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the X flag",                        GROUP_65816},
   { "sp",              KEY_SP,    "HEX", OPTION_ARG_OPTIONAL, "Initial value of the Stack Pointer register",       GROUP_65816},
   { 0 }
};

static int parse_skew(char *arg, struct argp_state *state) {
   int skew = 0;
   if (arg && strlen(arg) > 0) {
      skew = strtol(arg, (char **)NULL, 10);
      if (skew < -MAX_SKEW_VALUE || skew > MAX_SKEW_VALUE) {
         argp_error(state, "specified skew exceeds skew buffer size");
      }
   }
   return skew;
}

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
   int i;
   arguments_t *arguments = state->input;

   // First, pass argument to the profiler moduler
   profiler_parse_opt(key, arg, state);

   switch (key) {
   case KEY_DATA:
      arguments->idx_data = atoi(arg);
      break;
   case KEY_RNW:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rnw = atoi(arg);
      } else {
         arguments->idx_rnw = UNDEFINED;
      }
      break;
   case KEY_SYNC:
      if (arg && strlen(arg) > 0) {
         arguments->idx_sync = atoi(arg);
      } else {
         arguments->idx_sync = UNDEFINED;
      }
      break;
   case KEY_RDY:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rdy = atoi(arg);
      } else {
         arguments->idx_rdy = UNDEFINED;
      }
      break;
   case KEY_PHI1:
      if (arg && strlen(arg) > 0) {
         arguments->idx_phi1 = atoi(arg);
      } else {
         arguments->idx_phi1 = UNDEFINED;
      }
      break;
   case KEY_PHI2:
      if (arg && strlen(arg) > 0) {
         arguments->idx_phi2 = atoi(arg);
      } else {
         arguments->idx_phi2 = UNDEFINED;
      }
      break;
   case KEY_USER:
      if (arg && strlen(arg) > 0) {
         arguments->idx_user = atoi(arg);
      } else {
         arguments->idx_user = UNDEFINED;
      }
      break;
   case KEY_RST:
      if (arg && strlen(arg) > 0) {
         arguments->idx_rst = atoi(arg);
      } else {
         arguments->idx_rst = UNDEFINED;
      }
      break;
   case KEY_VECRST:
      if (arg && strlen(arg) > 0) {
         arguments->vec_rst = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->vec_rst = UNDEFINED;
      }
      break;
   case KEY_BBCTUBE:
      arguments->bbctube = 1;
      break;
   case KEY_VDA:
      if (arg && strlen(arg) > 0) {
         arguments->idx_vda = atoi(arg);
      } else {
         arguments->idx_vda = UNDEFINED;
      }
      break;
   case KEY_VPA:
      if (arg && strlen(arg) > 0) {
         arguments->idx_vpa = atoi(arg);
      } else {
         arguments->idx_vpa = UNDEFINED;
      }
      break;
   case KEY_E:
      if (arg && strlen(arg) > 0) {
         arguments->idx_e = atoi(arg);
      } else {
         arguments->idx_e = UNDEFINED;
      }
      break;
   case KEY_EMUL:
      if (arg && strlen(arg) > 0) {
         arguments->e_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->e_flag = UNDEFINED;
      }
      break;
   case KEY_SP:
      if (arg && strlen(arg) > 0) {
         arguments->sp_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->sp_reg = UNDEFINED;
      }
      break;
   case KEY_PB:
      if (arg && strlen(arg) > 0) {
         arguments->pb_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->pb_reg = UNDEFINED;
      }
      break;
   case KEY_DB:
      if (arg && strlen(arg) > 0) {
         arguments->db_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->db_reg = UNDEFINED;
      }
      break;
   case KEY_DP:
      if (arg && strlen(arg) > 0) {
         arguments->dp_reg = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->dp_reg = UNDEFINED;
      }
      break;
   case KEY_MS:
      if (arg && strlen(arg) > 0) {
         arguments->ms_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->ms_flag = UNDEFINED;
      }
      break;
   case KEY_XS:
      if (arg && strlen(arg) > 0) {
         arguments->xs_flag = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->xs_flag = UNDEFINED;
      }
      break;
   case KEY_SKIP:
      if (arg && strlen(arg) > 0) {
         arguments->skip = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->skip = 0;
      }
      break;
   case KEY_SKEW:
      arguments->skew_rd = parse_skew(arg, state);
      arguments->skew_wr = arguments->skew_rd;
      break;
   case KEY_SKEW_RD:
      arguments->skew_rd = parse_skew(arg, state);
      break;
   case KEY_SKEW_WR:
      arguments->skew_wr = parse_skew(arg, state);
      break;
   case KEY_LABELS:
      arguments->labels_file = arg;
      break;
   case KEY_MEM:
      if (arg && strlen(arg) > 0) {
         arguments->mem_model = strtol(arg, (char **)NULL, 16);
      } else {
         arguments->mem_model = 0;
      }
      break;
   case KEY_CPU:
      if (arg && strlen(arg) > 0) {
         i = 0;
         while (cpu_names[i].cpu_name) {
            if (strcasecmp(arg, cpu_names[i].cpu_name) == 0) {
               arguments->cpu_type = cpu_names[i].cpu_type;
               return 0;
            }
            i++;
         }
      }
      argp_error(state, "unsupported cpu type: %s", arg);
      break;
   case KEY_MACHINE:
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
   case KEY_DEBUG:
      arguments->debug = atoi(arg);
      break;
   case KEY_BYTE:
      arguments->byte = 1;
      break;
   case KEY_QUIET:
      arguments->show_address = 0;
      arguments->show_hex = 0;
      arguments->show_instruction = 0;
      arguments->show_state = 0;
      arguments->show_bbcfwa = 0;
      arguments->show_cycles = 0;
      arguments->show_samplenums = 0;
      break;
   case KEY_ADDR:
      arguments->show_address = 1;
      break;
   case KEY_SHOWROM:
      arguments->show_romno = 1;
      break;
   case KEY_HEX:
      arguments->show_hex = 1;
      break;
   case KEY_INSTR:
      arguments->show_instruction = 1;
      break;
   case KEY_STATE:
      arguments->show_state = 1;
      break;
   case KEY_BBCFWA:
      arguments->show_bbcfwa = 1;
      break;
   case KEY_CYCLES:
      arguments->show_cycles = 1;
      break;
   case KEY_SAMPLES:
      arguments->show_samplenums = 1;
      break;
   case KEY_PROFILE:
      arguments->profile = 1;
      break;
   case KEY_TRIGGER:
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
   case KEY_UNDOC:
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
         printf("%08x %2d %02x ", sample->sample_count, i, sample->data);
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
         if (sample->user >= 0) {
            putchar(' ');
            putchar('0' + sample->user);
         }
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

void write_hex8(char *buffer, int value) {
   write_hex1(buffer++, (value >> 28) & 15);
   write_hex1(buffer++, (value >> 24) & 15);
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
   int real_cycles;

   // This is a rather ugly hack to cope with a decode failure on Arlet's core.
   //
   if (arlet) {
      int op = sample_q->data;
      if (op == 0x08 || op == 0x48 || ((op == 0x5A || op == 0xDA) && (arguments.cpu_type == CPU_65C02_ARLET))) {
         // PHP, PHA, PHX, PHY
         //
         //    Normal 6502   Arlet
         // 0: 48 R SYNC     48 R SYNC <<<<< Push instruction
         // 1: XX R          XX R
         // 2: AA W          YY R
         // 3: XX R SYNC     AA W SYNC <<<<< Next instruction
         // 4: YY R          YY R
         //
         // Reorder the samples to make it look conventional:
         //   3->2
         //   1->3
         // But preserve the original type (sync) value, as this is correct
         sample_q[2].data = sample_q[3].data;
         sample_q[2].rnw  = sample_q[3].rnw;
         sample_q[3].data = sample_q[1].data;
         sample_q[3].rnw  = sample_q[1].rnw;
      }
      if (op == 0x28 || op == 0x68 || ((op == 0x7A || op == 0xFA) && (arguments.cpu_type == CPU_65C02_ARLET))) {
         // PLP, PLA, PLX, PLY
         //
         //    Normal 6502   Arlet
         // 0: 68 R SYNC     68 R SYNC <<<<< Pull instruction
         // 1: XX R          XX R
         // 2: ?? R          YY R
         // 3: AA R          AA R
         // 4: XX R SYNC     YY R SYNC <<<<< Next instruction
         // 5: YY R          YY R
         //
         // Reorder the samples to make it look conventional
         //    1->4
         // But preserve the original type (sync) value, as this is correct
         sample_q[4].data = sample_q[1].data;
      }
   }

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

   real_cycles = sample_q[num_cycles].cycle_count - sample_q[0].cycle_count;

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
      profiler_profile_instruction(instruction.pc, instruction.opcode, instruction.op1, instruction.op2, real_cycles);
   }

   int fail = em->get_and_clear_fail();

   // Try to minimise the calls to printf as these are quite expensive

   char *bp = disbuf;

   if ((fail | arguments.show_something) && triggered && !skipping_interrupted) {
      int numchars = 0;
      // Show sample count
      if (arguments.show_samplenums) {
         write_hex8(bp, sample_q->sample_count);
         bp += 8;
         *bp++ = ' ';
         *bp++ = ':';
         *bp++ = ' ';
      }
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
         if (arguments.show_romno) {
            bp += write_bankid(bp, pc);
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
         write_hex1(bp++, real_cycles);
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
      // Show the user defined signal value
      if (arguments.idx_user >= 0) {
         *bp++ = ' ';
         *bp++ = ':';
         *bp++ = ' ';
         int user = sample_q[num_cycles - 1].user;
         if (user >= 0) {
            *bp++ = '0' + user;
         } else {
            *bp++ = '?';
         }
      }
      // Show any errors
      if (fail) {
         bp += write_s(bp, " prediction failed");
      }
      // End the line
      *bp++ = 0;
      puts(disbuf);

   }

   total_cycles += real_cycles;
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

   if (sample_q[0].rst < 0) {
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
         rst_seen = cpu_rst_delay[arguments.cpu_type];
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

   // This helped when clock noise affected Arlet's core
   // (a better fix was to add 100pF cap to the clock)
   //
   // if (index > 0 && sample_q[index - 1].type == OPCODE && sample->type == OPCODE) {
   //    printf("Skipping duplicate SYNC\n");
   //    return;
   // }

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

static int min(int a, int b) {
   return (a < b) ? a : b;
}

static int max(int a, int b) {
   return (a > b) ? a : b;
}

static inline sample_type_t build_sample_type(uint16_t sample, int idx_vpa, int idx_vda, int idx_sync) {
   if (c816) {
      if (idx_vpa < 0 || idx_vda < 0) {
         return UNKNOWN;
      } else if ((sample >> idx_vpa) & 1) {
         if ((sample >> idx_vda) & 1) {
            return OPCODE;
         } else {
            return PROGRAM;
         }
      } else {
         if ((sample >> idx_vda) & 1) {
            return DATA;
         } else {
            return INTERNAL;
         }
      }
   } else {
      if (idx_sync < 0) {
         return UNKNOWN;
      } else if ((sample >> idx_sync) & 1) {
         return OPCODE;
      } else {
         return DATA;
      }
   }
}

void decode(FILE *stream) {

   // Pin mappings into the 16 bit words
   int idx_data  = arguments.idx_data;
   int idx_rnw   = arguments.idx_rnw ;
   int idx_sync  = arguments.idx_sync;
   int idx_rdy   = arguments.idx_rdy ;
   int idx_user  = arguments.idx_user;
   int idx_rst   = arguments.idx_rst;
   int idx_vda   = arguments.idx_vda;
   int idx_vpa   = arguments.idx_vpa;
   int idx_e     = arguments.idx_e;

   // Invert RDY polarity on the 6800 to allow it to be driven from BA
   int rdy_pol = (arguments.cpu_type == CPU_6800) ? 0 : 1;

   // Handle clock inversion of phi1 used rather than phi2
   int idx_phi = -1;
   int clk_pol = 0;
   if (arguments.idx_phi1 >= 0) {
      idx_phi = arguments.idx_phi1;
      clk_pol = 1;
   } else if (arguments.idx_phi2 >= 0) {
      idx_phi = arguments.idx_phi2;
   }

   // The structured bus sample we will pass on to the next level of processing
   sample_t s;

   // Skip the start of the file, if required
   if (arguments.skip) {
      fseek(stream, arguments.skip * (arguments.byte ? 1 : 2), SEEK_SET);
   }

   // Common to all sampling modes
   s.type = UNKNOWN;
   s.sample_count = 1;
   s.cycle_count = 1;
   s.rnw  = -1;
   s.rst  = -1;
   s.e    = -1;
   s.user = -1;

   if (arguments.byte) {

      // ------------------------------------------------------------
      // Synchronous byte sampling mode
      // ------------------------------------------------------------

      // In byte mode we have only data bus samples, nothing else so we must
      // use the sync-less decoder. All the control signals should be marked
      // as disconnected, by being set to -1.

      // Read the capture file, and queue structured sampled for the decoder
      int num;
      while ((num = fread(buffer8, sizeof(uint8_t), BUFSIZE, stream)) > 0) {
         uint8_t *sampleptr = &buffer8[0];
         while (num-- > 0) {
            s.data = *sampleptr++;
            queue_sample(&s);
            s.sample_count++;
            s.cycle_count++;
         }
      }

   } else if (idx_phi < 0 ) {

      // ------------------------------------------------------------
      // Synchronous word sampling mode
      // ------------------------------------------------------------

      // In word sampling mode we have data bus samples, plus
      // optionally rnw, sync, rdy, phy2 and rst.

      // In synchronous word sampling mode clke is not connected, and
      // it's assumed that each sample represents a seperate bus
      // cycle.

      // Read the capture file, and queue structured sampled for the decoder
      int num;
      while ((num = fread(buffer, sizeof(uint16_t), BUFSIZE, stream)) > 0) {
         uint16_t *sampleptr = &buffer[0];
         while (num-- > 0) {
            uint16_t sample = *sampleptr++;
            // Drop samples where RDY=0 (or BA=1 for 6800)
            if (idx_rdy < 0 || (((sample >> idx_rdy) & 1) == rdy_pol)) {
               s.type = build_sample_type(sample, idx_vpa, idx_vda, idx_sync);
               if (idx_rnw >= 0) {
                  s.rnw = (sample >> idx_rnw ) & 1;
               }
               if (idx_rst >= 0) {
                  s.rst = (sample >> idx_rst) & 1;
               }
               if (idx_e >= 0) {
                  s.e = (sample >> idx_e) & 1;
               }
               if (idx_user >= 0) {
                  s.user = (sample >> idx_user) & 1;
               }
               s.data = (sample >> idx_data) & 255;
               queue_sample(&s);
            }
            s.sample_count++;
            s.cycle_count++;
         }
      }

   } else {

      // ------------------------------------------------------------
      // Asynchronous word sampling mode
      // ------------------------------------------------------------

      // In word sampling mode we have data bus samples, plus
      // optionally rnw, sync, rdy, phy2 and rst.

      // In asynchronous word sampling mode clke is connected, and
      // the capture file contans multple samples per bus cycle.

      // The previous value of clke, to detect the rising/falling edge
      int last_phi2 = -1;

      // A small circular buffer for skewing the sampling of the data bus
      uint16_t skew_buffer  [SKEW_BUFFER_SIZE];

      // Minimize the amount of buffering to avoid unnecessary garbage
      int min_skew = min(arguments.skew_rd, arguments.skew_wr);
      int max_skew = max(arguments.skew_rd, arguments.skew_wr);

      int tail        = max(0, max_skew);
      int head        = 0;
      int rddata_head = arguments.skew_rd;
      int wrdata_head = arguments.skew_wr;

      // If any of the skews are negative, then increase all by this amount so they are all positive
      if (min_skew < 0) {
         tail        = (       tail - min_skew) & (SKEW_BUFFER_SIZE - 1);
         head        = (       head - min_skew) & (SKEW_BUFFER_SIZE - 1);
         rddata_head = (rddata_head - min_skew) & (SKEW_BUFFER_SIZE - 1);
         wrdata_head = (wrdata_head - min_skew) & (SKEW_BUFFER_SIZE - 1);
      }

      // Clear the buffer, so the first few samples are ignored
      for (int i = 0; i < SKEW_BUFFER_SIZE; i++) {
         skew_buffer[i] = 0;
      }

      // Read the capture file, and queue structured sampled for the decoder
      int num;
      while ((num = fread(buffer, sizeof(uint16_t), BUFSIZE, stream)) > 0) {
         uint16_t *sampleptr = &buffer[0];
         while (num-- > 0) {
            skew_buffer[tail] = *sampleptr++;
            uint16_t sample   = skew_buffer[head];
            // Only act on edges of phi
            int pin_phi2 = clk_pol ^ ((sample >> idx_phi) & 1);
            if (pin_phi2 != last_phi2) {
               last_phi2 = pin_phi2;
               if (pin_phi2) {
                  // Sample control signals after rising edge of PHI2
                  // Note: this is a change for the 65816, but should be fine timing wise
                  s.type = build_sample_type(sample, idx_vpa, idx_vda, idx_sync);
                  if (idx_rnw >= 0) {
                     s.rnw = (sample >> idx_rnw ) & 1;
                  }
                  if (idx_rst >= 0) {
                     s.rst = (sample >> idx_rst) & 1;
                     }
                  if (idx_e >= 0) {
                     s.e = (sample >> idx_e) & 1;
                  }
                  if (idx_user >= 0) {
                     s.user = (sample >> idx_user) & 1;
                  }
               } else {
                  if (idx_rdy < 0 || ((sample >> idx_rdy) & 1)) {
                     // Sample the data skewed (--skew=) relative to the falling edge of PHI2
                     s.data = (skew_buffer[s.rnw == 0 ? wrdata_head : rddata_head] >> idx_data) & 255;
                     queue_sample(&s);
                  }
                  s.cycle_count++;
               }
            }
            s.sample_count++;
            // Increment the circular buffer pointers in lock-step to keey the skew constant
            tail        = (tail        + 1) & (SKEW_BUFFER_SIZE - 1);
            head        = (head        + 1) & (SKEW_BUFFER_SIZE - 1);
            rddata_head = (rddata_head + 1) & (SKEW_BUFFER_SIZE - 1);
            wrdata_head = (wrdata_head + 1) & (SKEW_BUFFER_SIZE - 1);
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
   // General options
   arguments.cpu_type         = CPU_UNKNOWN;
   arguments.machine          = MACHINE_DEFAULT;
   arguments.vec_rst          = UNSPECIFIED;
   arguments.sp_reg           = UNSPECIFIED;
   arguments.bbctube          = 0;
   arguments.byte             = 0;
   arguments.debug            = 0;
   arguments.mem_model        = 0;
   arguments.skip             = 0;
   arguments.skew_rd          = UNSPECIFIED;
   arguments.skew_wr          = UNSPECIFIED;
   arguments.profile          = 0;
   arguments.trigger_start    = UNSPECIFIED;
   arguments.trigger_stop     = UNSPECIFIED;
   arguments.trigger_skipint  = 0;
   arguments.filename         = NULL;

   // Output options
   arguments.show_address     = 1;
   arguments.show_hex         = 0;
   arguments.show_instruction = 1;
   arguments.show_state       = 0;
   arguments.show_bbcfwa      = 0;
   arguments.show_cycles      = 0;
   arguments.show_samplenums  = 0;

   // Signal definition options
   arguments.idx_data         = UNSPECIFIED;
   arguments.idx_rnw          = UNSPECIFIED;
   arguments.idx_sync         = UNSPECIFIED;
   arguments.idx_vpa          = UNSPECIFIED;
   arguments.idx_rdy          = UNSPECIFIED;
   arguments.idx_vda          = UNSPECIFIED;
   arguments.idx_e            = UNSPECIFIED;
   arguments.idx_rst          = UNSPECIFIED;
   arguments.idx_phi1         = UNSPECIFIED;
   arguments.idx_phi2         = UNSPECIFIED;
   arguments.idx_user         = UNSPECIFIED;

   // Additional 6502 options
   arguments.undocumented     = 0;

   // Additional 65816 options
   arguments.pb_reg           = UNSPECIFIED;
   arguments.db_reg           = UNSPECIFIED;
   arguments.dp_reg           = UNSPECIFIED;
   arguments.e_flag           = UNSPECIFIED;
   arguments.ms_flag          = UNSPECIFIED;
   arguments.xs_flag          = UNSPECIFIED;

   argp_parse(&argp, argc, argv, 0, 0, &arguments);

   if (arguments.trigger_start < 0) {
      triggered = 1;
   }

   arguments.show_something = arguments.show_samplenums | arguments.show_address | arguments.show_hex | arguments.show_instruction | arguments.show_state | arguments.show_bbcfwa | arguments.show_cycles;

   // Normally the data file should be 16 bit samples. In byte mode
   // the data file is 8 bit samples, and all the control signals are
   // assumed to be don't care.
   if (arguments.byte) {
      if (arguments.idx_rnw != UNSPECIFIED) {
         fprintf(stderr, "--rnw is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_sync != UNSPECIFIED) {
         fprintf(stderr, "--sync is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_phi1 != UNSPECIFIED) {
         fprintf(stderr, "--phi1 is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_phi2 != UNSPECIFIED) {
         fprintf(stderr, "--phi2 is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_rst != UNSPECIFIED) {
         fprintf(stderr, "--rst is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_rdy != UNSPECIFIED) {
         fprintf(stderr, "--rdy is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_vpa != UNSPECIFIED) {
         fprintf(stderr, "--vpa is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_vda != UNSPECIFIED) {
         fprintf(stderr, "--vda is incompatible with byte mode\n");
         return 1;
      }
      if (arguments.idx_e != UNSPECIFIED) {
         fprintf(stderr, "--e is incompatible with byte mode\n");
         return 1;
      }
   }

   // Apply Machine specific defaults
   if (arguments.vec_rst == UNSPECIFIED) {
      switch (arguments.machine) {
      case MACHINE_BEEB:
         arguments.vec_rst = 0xA9D9CD;
         break;
      case MACHINE_MASTER:
         arguments.vec_rst = 0xA9E364;
         break;
      case MACHINE_ELK:
         arguments.vec_rst = 0xA9D8D2;
         break;
      case MACHINE_ATOM:
         arguments.vec_rst = 0xA2FF3F;
         break;
      case MACHINE_MEK6800D2:
         arguments.vec_rst = 0x8E8DE0;
         break;
      default:
         arguments.vec_rst = 0xFFFFFF;
      }
   }
   if (arguments.cpu_type == CPU_UNKNOWN) {
      switch (arguments.machine) {
      case MACHINE_MASTER:
         arguments.cpu_type = CPU_65C02;
         break;
      case MACHINE_MEK6800D2:
         arguments.cpu_type = CPU_6800;
         break;
      default:
         arguments.cpu_type = CPU_6502;
         break;
      }
   }

   int memory_size;
   // Initialize memory modelling
   // (em->init actually mallocs the memory)
   if (arguments.cpu_type == CPU_65C816) {
      // 16MB
      memory_size = 0x1000000;
   } else {
      // 64KB
      memory_size = 0x10000;
   }

   memory_init(memory_size, arguments.machine, arguments.bbctube);

   // Turn on memory write logging if show rom bank option (-r) is selected
   if (arguments.show_romno) {
      arguments.mem_model |= (1 << MEM_DATA) | (1 << MEM_STACK);
   }

   memory_set_modelling(  arguments.mem_model       & 0x0f);
   memory_set_rd_logging((arguments.mem_model >> 4) & 0x0f);
   memory_set_wr_logging((arguments.mem_model >> 8) & 0x0f);

   // Load the swift format symbol file
   if (arguments.labels_file) {
      symbol_init(memory_size);
      symbol_import_swift(arguments.labels_file);
   }

   // Validate options compatibility with CPU
   if (arguments.cpu_type != CPU_6502 && arguments.cpu_type != CPU_6800 && arguments.undocumented) {
      fprintf(stderr, "--undocumented is only applicable to the 6502/6800\n");
      return 1;
   }
   if (arguments.cpu_type == CPU_65C816) {
      if (arguments.idx_sync != UNSPECIFIED) {
         fprintf(stderr, "--sync is not applicable to the 65C816\n");
         return 1;
      }
   } else {
      if (arguments.idx_vda != UNSPECIFIED) {
         fprintf(stderr, "--vda is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.idx_vpa != UNSPECIFIED) {
         fprintf(stderr, "--vpa is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.idx_e != UNSPECIFIED) {
         fprintf(stderr, "--e is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.pb_reg != UNSPECIFIED) {
         fprintf(stderr, "--pb is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.db_reg != UNSPECIFIED) {
         fprintf(stderr, "--db is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.dp_reg != UNSPECIFIED) {
         fprintf(stderr, "--dp is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.e_flag != UNSPECIFIED) {
         fprintf(stderr, "--emul is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.ms_flag != UNSPECIFIED) {
         fprintf(stderr, "--ms is only applicable to the 65C816\n");
         return 1;
      }
      if (arguments.xs_flag != UNSPECIFIED) {
         fprintf(stderr, "--xs is only applicable to the 65C816\n");
         return 1;
      }
   }

   // Implement default pins mapping for unspecified pins
   if (arguments.idx_data == UNSPECIFIED) {
      arguments.idx_data = 0;
   }
   if (arguments.idx_rnw == UNSPECIFIED) {
      arguments.idx_rnw = 8;
   }
   if (arguments.idx_sync == UNSPECIFIED) {
      arguments.idx_sync = 9;
   }
   if (arguments.idx_vpa == UNSPECIFIED) {
      arguments.idx_vpa = 9;
   }
   if (arguments.idx_rdy == UNSPECIFIED) {
      arguments.idx_rdy = 10;
   }
   if (arguments.idx_vda == UNSPECIFIED) {
      arguments.idx_vda = 11;
   }
   if (arguments.idx_e == UNSPECIFIED) {
      arguments.idx_e = 12;
   }
   if (arguments.idx_rst == UNSPECIFIED) {
      arguments.idx_rst = 14;
   }
   // Flag conflicting use of --phi1 and --phi2
   if (arguments.idx_phi1 >= 0 && arguments.idx_phi2 >= 0) {
      fprintf(stderr, "--phi1 and --phi2 cannot both be assigned to pins\n");
      return 1;
   }
   // Only default phi2 if phi1 is not assigned to a pin
   if (arguments.idx_phi2 == UNSPECIFIED && arguments.idx_phi1 < 0) {
      arguments.idx_phi2 = 15;
   }

   if (arguments.skew_rd == UNSPECIFIED) {
      switch (arguments.machine) {
      case MACHINE_BEEB:
         arguments.skew_rd =  0; // sample after Phi2 falls
         break;
      case MACHINE_MASTER:
         arguments.skew_rd = -1; // sample before PHI2 fails
         break;
      default:
         arguments.skew_rd = -1; // sample before PHI2 fails
         break;
      }
   }
   if (arguments.skew_wr == UNSPECIFIED) {
      switch (arguments.machine) {
      case MACHINE_BEEB:
         arguments.skew_wr = -1; // sample before PHI2 fails
         break;
      case MACHINE_MASTER:
         arguments.skew_wr = -2; // sample well before PHI2 fails
         break;
      default:
         arguments.skew_wr = -1; // sample before PHI2 fails
         break;
      }
   }

   c816 = 0;
   if (arguments.cpu_type == CPU_65C816) {
      c816 = 1;
      em = &em_65816;
   } else if (arguments.cpu_type == CPU_6800) {
      em = &em_6800;
   } else {
      em = &em_6502;
   }

   arlet = (arguments.cpu_type == CPU_6502_ARLET || arguments.cpu_type == CPU_65C02_ARLET);

   em->init(&arguments);

   if (arguments.profile) {
      profiler_init(em);
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

   decode(stream);
   fclose(stream);

   if (arguments.profile) {
      profiler_done();
   }

   return 0;
}
