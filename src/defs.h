#ifndef DEFS
#define DEFS

#include <inttypes.h>

// Sample Queue Depth - needs to fit the longest instruction
#define DEPTH 10

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
   sample_type_t type;
   uint8_t       data;
   int8_t         rnw; // -1 indicates unknown
   int8_t         rst; // -1 indicates unknown
} sample_t;


typedef struct {
   int           pc;
   uint8_t       opcode;
   uint8_t       op1;
   uint8_t       op2;
   uint8_t       op3;
   uint8_t       opcount;
} instruction_t;

void write_hex1(char *buffer, int value);
void write_hex2(char *buffer, int value);


typedef struct {
   void (*init)(int support_c02, int support_rockwell, int support_undocumented, int decode_bbctube, int mast_nordy);
   int (*match_interrupt)(sample_t *sample_q, int num_samples);
   int (*count_cycles)(sample_t *sample_q, int intr_seen);
   void (*reset)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   void (*interrupt)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   void (*emulate)(sample_t *sample_q, int num_cycles, instruction_t *instruction);
   int (*disassemble)(instruction_t *instruction);
   int (*get_PC)();
   int (*read_memory)(int address);
   char *(*get_state)();
   int (*get_and_clear_fail)();
} cpu_emulator_t;


#endif
