#ifndef _INCLUDE_EM_6502_H
#define _INCLUDE_EM_6502_H

#include "defs.h"

void em_init(int support_c02, int support_rockwell, int support_undocumented, int decode_bbctube);

int em_count_cycles(sample_t *sample_q, int rst_seen, int intr_seen);

int em_match_reset(sample_t *sample_q, int vec_rst);

int em_match_interrupt(sample_t *sample_q);

void em_reset(sample_t *sample_q, instruction_t *instruction);

void em_interrupt(sample_t *sample_q, instruction_t *instruction);

void em_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction);

int em_disassemble(instruction_t *instruction);

int em_get_PC();

int em_read_memory(int address);

char *em_get_state();

int em_get_and_clear_fail();


#endif
