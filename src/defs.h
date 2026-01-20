#ifndef DEFS
#define DEFS

#include <inttypes.h>

typedef enum {
   MACHINE_DEFAULT,
   MACHINE_BEEB,
   MACHINE_MASTER,
   MACHINE_ELK,
   MACHINE_ATOM,
   MACHINE_MEK6800D2,
   MACHINE_BLITTER,
} machine_t;

typedef enum {
   CPU_UNKNOWN,
   CPU_6502,
   CPU_6502_ARLET,
   CPU_65C02,
   CPU_65C02_ROCKWELL,
   CPU_65C02_WDC,
   CPU_65C02_ARLET,
   CPU_65C02_ALAND,
   CPU_65C816,
   CPU_6800,
} cpu_t;

// Sample Queue Depth - needs to fit the longest instruction
#define DEPTH 13

// Sample_type_t is an abstraction of both the 6502 SYNC and the 65816 VDA/VPA

typedef enum {     // 6502 Sync    65815 VDA/VPA
   UNKNOWN,        //      ?             ?   ?
   INTERNAL,       //      -             0   0
   PROGRAM,        //      -             0   1
   DATA,           //      0             1   0
   OPCODE,         //      1             1   1
   LAST            // a marker for the end of stream
} sample_type_t;

typedef struct {
   uint32_t      sample_count;
   uint32_t      cycle_count;
   sample_type_t type;
   uint8_t       data;
   int8_t         rnw; // -1 indicates unknown
   int8_t         rst; // -1 indicates unknown
   int8_t           e; // -1 indicates unknown (65816 e pin)
   int8_t        user; // -1 indicates unknown (user defined signal)
} sample_t;


typedef struct {
   int           pc;
   int           pb;
   uint8_t       opcode;
   uint8_t       op1;
   uint8_t       op2;
   uint8_t       op3;
   uint8_t       opcount;
} instruction_t;

void write_hex1(char *buffer, int value);
void write_hex2(char *buffer, int value);
void write_hex4(char *buffer, int value);
void write_hex6(char *buffer, int value);
int  write_s   (char *buffer, const char *s);

typedef struct {
   cpu_t cpu_type;
   machine_t machine;
   int idx_data;
   int idx_rnw;
   int idx_sync;
   int idx_rdy;
   int idx_phi1;
   int idx_phi2;
   int idx_user;
   int idx_rst;
   int idx_vda;
   int idx_vpa;
   int idx_e;
   int vec_rst;
   int show_address;
   int show_hex;
   int show_instruction;
   int show_state;
   int show_bbcfwa;
   int show_cycles;
   int show_samplenums;
   int show_something;
   int bbctube;
   int undocumented;
   int e_flag;
   int ms_flag;
   int xs_flag;
   int sp_reg;
   int pb_reg;
   int db_reg;
   int dp_reg;
   int byte;
   int debug;
   int skip;
   int skew_rd;
   int skew_wr;
   char *labels_file;
   int mem_model;
   int profile;
   int trigger_start;
   int trigger_stop;
   int trigger_skipint;
   char *filename;
   int show_romno;
} arguments_t;

typedef struct {
   void (*init)(arguments_t *args);
   int (*match_interrupt)(sample_t *sample_q, int num_samples);
   int (*count_cycles)(sample_t *sample_q, int intr_seen);
   void (*reset)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   void (*interrupt)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   void (*emulate)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   int (*disassemble)(char *bp, instruction_t *instruction);
   int (*get_PC)();
   int (*get_PB)();
   int (*read_memory)(int address);
   char *(*get_state)(char*);
   int (*get_and_clear_fail)();
} cpu_emulator_t;

extern int failflag;

#endif
