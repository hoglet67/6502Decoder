#ifndef _INCLUDE_EM_6502_H
#define _INCLUDE_EM_6502_H

void em_init(int support_c02, int support_undocumented);

void em_reset();

void em_interrupt(int operand);

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

char *em_get_state();

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
   void (*emulate)(int);
   int len;
   const char *fmt;
} InstrType;

extern InstrType *instr_table;

#endif
