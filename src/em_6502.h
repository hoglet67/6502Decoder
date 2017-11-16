#ifndef _INCLUDE_EM_6502_H
#define _INCLUDE_EM_6502_H

void em_interrupt(int operand);

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
   BRANCHOP
} OpType;

typedef struct {
   const char *mnemonic;
   AddrMode mode;
   OpType optype;
   void (*emulate)(int);
} InstrType;

extern InstrType instr_table[];

extern int addr_mode_len_map[];

extern char *addr_mode_format_map[];

#endif
