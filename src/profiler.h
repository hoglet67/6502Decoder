#ifndef _INCLUDE_PROFILER_H
#define _INCLUDE_PROFILER_H

#include <argp.h>
#include <inttypes.h>


// Slot for instructions that fall outside the region of interest
// (don't change this or lots of things will break!)
#define OTHER_CONTEXT    0x10000

// Maximum length of bar of asterisks
#define BAR_WIDTH 50

// Flags are currently specific to the block profiler
#define FLAG_IMP           1
#define FLAG_JSR           2
#define FLAG_JMP           4
#define FLAG_BB_TAKEN      8
#define FLAG_FB_TAKEN     16
#define FLAG_BB_NOT_TAKEN 32
#define FLAG_FB_NOT_TAKEN 64

// A common data type, useful for several of the profilers
typedef struct {
   uint32_t cycles;
   uint32_t instructions;
   uint32_t calls;
   int flags;
} address_t;

// All profiler instance data should start with this type

typedef struct {
   const char *name;
   const char *arg;
   void                (*init)(void *ptr);
   void (*profile_instruction)(void *ptr, int pc, int opcode, int op1, int op2, int num_cycles);
   void                (*done)(void *ptr);
} profiler_t;

// Public methods, called from main program

void profiler_parse_opt(int key, char *arg, struct argp_state *state);
void profiler_init();
void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles);
void profiler_done();

// Helper methods, for use by profiler implementations

void profiler_output_helper(address_t *profile_counts, int show_bars, int show_other);

#endif
