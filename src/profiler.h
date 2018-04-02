#ifndef _INCLUDE_PROFILER_H
#define _INCLUDE_PROFILER_H

void profiler_parse_args(char *arg);

void profiler_init();

void profiler_instruction(int pc, int opcode, int op1, int op2, int num_cycles);

void profiler_done();

#endif
