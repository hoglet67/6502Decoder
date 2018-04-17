#ifndef _INCLUDE_PROFILER_H
#define _INCLUDE_PROFILER_H

#include <argp.h>

void profiler_parse_opt(int key, char *arg, struct argp_state *state);

void profiler_init();

void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles);

void profiler_done();

typedef struct profiler {
   const char *name;
   const char *arg;
   void                (*init)(void *ptr);
   void (*profile_instruction)(void *ptr, int pc, int opcode, int op1, int op2, int num_cycles);
   void                (*done)(void *ptr);
} profiler_t;

#endif
