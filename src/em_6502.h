#ifndef _INCLUDE_EM_6502_H
#define _INCLUDE_EM_6502_H

#include "defs.h"

void em_init(int support_c02, int support_rockwell, int support_undocumented, int decode_bbctube);

int em_count_cycles(sample_t *sample_q);

int em_match_reset(sample_t *sample_q, int vec_rst);

int em_match_interrupt(sample_t *sample_q);

void em_reset(sample_t *sample_q, instruction_t *instruction);

void em_interrupt(sample_t *sample_q, instruction_t *instruction);

void em_emulate(sample_t *sample_q, int num_cycles, instruction_t *instruction);

int em_disassemble(instruction_t *instruction);

int compare_NVDIZC(int operand);

int em_get_N();

int em_get_V();

int em_get_D();

int em_get_I();

int em_get_Z();

int em_get_C();

int em_get_A();

int em_get_X();

int em_get_Y();

int em_get_S();

int em_get_PC();

int em_read_memory(int address);

char *em_get_state();

char *em_get_fwa(int a_sign, int a_exp, int a_mantissa, int a_round, int a_overflow);

int em_get_and_clear_fail();

typedef enum {
   IMP,
   IMPA,
   BRA,
   IMM,
   ZP,
   ZPX,
   ZPY,
   INDX,
   INDY,
   IND,
   ABS,
   ABSX,
   ABSY,
   IND16,
   IND1X,
   ZPR
} AddrMode ;

typedef enum {
   READOP,
   WRITEOP,
   RMWOP,
   TSBTRBOP,
   BRANCHOP
} OpType;

typedef struct {
   int len;
   const char *fmt;
} AddrModeType;

typedef struct {
   const char *mnemonic;
   int undocumented;
   AddrMode mode;
   int cycles;
   int decimalcorrect;
   OpType optype;
   void (*emulate)(int, int);
   int len;
   const char *fmt;
} InstrType;

extern InstrType *instr_table;

#endif
