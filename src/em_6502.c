#include <stdio.h>
#include <string.h>
#include "em_6502.h"

AddrModeType addr_mode_table[] = {
   {1,    "%1$s"},                  // IMP
   {1,    "%1$s A"},                // IMPA
   {2,    "%1$s %2$s"},             // BRA
   {2,    "%1$s #%2$02X"},          // IMM
   {2,    "%1$s %2$02X"},           // ZP
   {2,    "%1$s %2$02X,X"},         // ZPX
   {2,    "%1$s %2$02X,Y"},         // ZPY
   {2,    "%1$s (%2$02X,X)"},       // INDX
   {2,    "%1$s (%2$02X),Y"},       // INDY
   {2,    "%1$s (%2$02X)"},         // IND
   {3,    "%1$s %3$02X%2$02X"},     // ABS
   {3,    "%1$s %3$02X%2$02X,X"},   // ABSX
   {3,    "%1$s %3$02X%2$02X,Y"},   // ABSY
   {3,    "%1$s (%3$02X%2$02X)"},   // IND1
   {3,    "%1$s (%3$02X%2$02X,X)"}, // IND1X
   {3,    "%1$s %2$02X,%3$s"}       //
};

const char default_state[] = "A=?? X=?? Y=?? SP=?? N=? V=? D=? I=? Z=? C=?";

#define OFFSET_A   2
#define OFFSET_X   7
#define OFFSET_Y  12
#define OFFSET_S  18
#define OFFSET_N  23
#define OFFSET_V  27
#define OFFSET_D  31
#define OFFSET_I  35
#define OFFSET_Z  39
#define OFFSET_C  43
#define OFFSET_FF 44

static char buffer[80];

// 6502 registers: -1 means unknown
static int A = -1;
static int X = -1;
static int Y = -1;
static int S = -1;
// 6502 flags: -1 means unknown
static int N = -1;
static int V = -1;
static int D = -1;
static int I = -1;
static int Z = -1;
static int C = -1;
// indicate state prediction failed
static int failflag = 0;


static void op_STA(int operand);
static void op_STX(int operand);
static void op_STY(int operand);

static void check_NVDIZC(int operand) {
   if (N >= 0) {
      if (N != ((operand >> 7) & 1)) {
         failflag = 1;
      }
   }
   if (V >= 0) {
      if (V != ((operand >> 6) & 1)) {
         failflag = 1;
      }
   }
   if (D >= 0) {
      if (D != ((operand >> 3) & 1)) {
         failflag = 1;
      }
   }
   if (I >= 0) {
      if (I != ((operand >> 2) & 1)) {
         failflag = 1;
      }
   }
   if (Z >= 0) {
      if (Z != ((operand >> 1) & 1)) {
         failflag = 1;
      }
   }
   if (C >= 0) {
      if (C != ((operand >> 0) & 1)) {
         failflag = 1;
      }
   }
}

static void set_NVDIZC(int operand) {
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   D = (operand >> 3) & 1;
   I = (operand >> 2) & 1;
   Z = (operand >> 1) & 1;
   C = (operand >> 0) & 1;
}

static void set_NZ_unknown() {
   N = -1;
   Z = -1;
}

static void set_NZC_unknown() {
   N = -1;
   Z = -1;
   C = -1;
}

static void set_NVZC_unknown() {
   N = -1;
   V = -1;
   Z = -1;
   C = -1;
}

static void set_NZ(int value) {
   N = (value & 128) > 0;
   Z = value == 0;
}

void em_interrupt(int operand) {
   if (S >= 0) {
      S = (S - 3) & 255;
   }
   check_NVDIZC(operand);
   set_NVDIZC(operand);
   I = 1;
   D = 0; // TODO: 65C02 only
}
static void write_hex1(char *buffer, int value) {
   *buffer = value + (value < 10 ? '0' : 'A' - 10);
}

static void write_hex2(char *buffer, int value) {
   write_hex1(buffer++, (value >> 4) & 15);
   write_hex1(buffer++, (value >> 0) & 15);
}

int em_get_N() {
   return N;
}

int em_get_V() {
   return V;
}

int em_get_Z() {
   return Z;
}

int em_get_C() {
   return C;
}

int em_get_A() {
   return A;
}

int em_get_X() {
   return X;
}

int em_get_Y() {
   return Y;
}

int em_get_S() {
   return S;
}

char *em_get_state() {
   strcpy(buffer, default_state);
   if (A >= 0) {
      write_hex2(buffer + OFFSET_A, A);
   }
   if (X >= 0) {
      write_hex2(buffer + OFFSET_X, X);
   }
   if (Y >= 0) {
      write_hex2(buffer + OFFSET_Y, Y);
   }
   if (S >= 0) {
      write_hex2(buffer + OFFSET_S, S);
   }
   if (N >= 0) {
      buffer[OFFSET_N] = '0' + N;
   }
   if (V >= 0) {
      buffer[OFFSET_V] = '0' + V;
   }
   if (D >= 0) {
      buffer[OFFSET_D] = '0' + D;
   }
   if (I >= 0) {
      buffer[OFFSET_I] = '0' + I;
   }
   if (Z >= 0) {
      buffer[OFFSET_Z] = '0' + Z;
   }
   if (C >= 0) {
      buffer[OFFSET_C] = '0' + C;
   }
   if (failflag) {
      sprintf(buffer + OFFSET_FF, " prediction failed");
   }
   failflag = 0;
   return buffer;
}


static void op_ADC(int operand) {
   // TODO: Decimal mode
   if (A >= 0 && C >= 0) {
      int tmp = A + operand + C;
      C = (tmp >> 8) & 1;
      V = (((A ^ operand) & 0x80) == 0) && (((A ^ tmp) & 0x80) != 0);
      A = tmp & 255;
      set_NZ(A);
   } else {
      A = -1;
      set_NVZC_unknown();
   }
}

static void op_AND(int operand) {
   if (A >= 0) {
      A = A & operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_ASLA(int operand) {
   if (A >= 0) {
      C = (A >> 7) & 1;
      A = (A << 1) & 255;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_ASL(int operand) {
   C = (operand >> 7) & 1;
   int tmp = (operand << 1) & 255;
   set_NZ(tmp);
}

static void op_BCC(int branch_taken) {
   if (C >= 0) {
      if (C == branch_taken) {
         failflag = 1;
      }
   } else {
      C = 1 - branch_taken;
   }
}

static void op_BCS(int branch_taken) {
   if (C >= 0) {
      if (C != branch_taken) {
         failflag = 1;
      }
   } else {
      C = branch_taken;
   }
}

static void op_BNE(int branch_taken) {
   if (Z >= 0) {
      if (Z == branch_taken) {
         failflag = 1;
      }
   } else {
      Z = 1 - branch_taken;
   }
}

static void op_BEQ(int branch_taken) {
   if (Z >= 0) {
      if (Z != branch_taken) {
         failflag = 1;
        }
   } else {
      Z = branch_taken;
   }
}

static void op_BPL(int branch_taken) {
   if (N >= 0) {
      if (N == branch_taken) {
         failflag = 1;
      }
   } else {
      N = 1 - branch_taken;
   }
}

static void op_BMI(int branch_taken) {
   if (N >= 0) {
      if (N != branch_taken) {
         failflag = 1;
      }
   } else {
      N = branch_taken;
   }
}

static void op_BVC(int branch_taken) {
   if (V >= 0) {
      if (V == branch_taken) {
         failflag = 1;
      }
   } else {
      V = 1 - branch_taken;
   }
}

static void op_BVS(int branch_taken) {
   if (V >= 0) {
      if (V != branch_taken) {
         failflag = 1;
        }
   } else {
      V = branch_taken;
   }
}

static void op_BRK(int operand) {
   em_interrupt(operand);
}

static void op_BIT_IMM(int operand) {
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_BIT(int operand) {
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_CLC(int operand) {
   C = 0;
}

static void op_CLD(int operand) {
   D = 0;
}

static void op_CLI(int operand) {
   I = 0;
}

static void op_CLV(int operand) {
   V = 0;
}

static void op_CMP(int operand) {
   if (A >= 0) {
      int tmp = A - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   }
}

static void op_CPX(int operand) {
   if (X >= 0) {
      int tmp = X - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   }
}

static void op_CPY(int operand) {
   if (Y >= 0) {
      int tmp = Y - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   }
}

static void op_DECA(int operand) {
   if (A >= 0) {
      A = (A - 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEC(int operand) {
   int tmp = (operand - 1) & 255;
   set_NZ(tmp);
}

static void op_DEX(int operand) {
   if (X >= 0) {
      X = (X - 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEY(int operand) {
   if (Y >= 0) {
      Y = (Y - 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_EOR(int operand) {
   if (A >= 0) {
      A = A ^ operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INCA(int operand) {
   if (A >= 0) {
      A = (A + 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INC(int operand) {
   int tmp = (operand + 1) & 255;
   set_NZ(tmp);
}

static void op_INX(int operand) {
   if (X >= 0) {
      X = (X + 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_INY(int operand) {
   if (Y >= 0) {
      Y = (Y + 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_JSR(int operand) {
   if (S >= 0) {
      S = (S - 2) & 255;
   }
}

static void op_LDA(int operand) {
   A = operand;
   set_NZ(A);
}

static void op_LDX(int operand) {
   X = operand;
   set_NZ(X);
}

static void op_LDY(int operand) {
   Y = operand;
   set_NZ(Y);
}

static void op_LSRA(int operand) {
   if (A >= 0) {
      C = A & 1;
      A = A >> 1;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_LSR(int operand) {
   C = operand & 1;
   int tmp = operand >> 1;
   set_NZ(tmp);
}

static void op_ORA(int operand) {
   if (A >= 0) {
      A = A | operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_PHA(int operand) {
   if (S >= 0) {
      S = (S - 1) & 255;
   }
   op_STA(operand);
}

static void op_PHP(int operand) {
   if (S >= 0) {
      S = (S - 1) & 255;
   }
   check_NVDIZC(operand);
   set_NVDIZC(operand);
}

static void op_PHX(int operand) {
   if (S >= 0) {
      S = (S - 1) & 255;
   }
   op_STX(operand);
}

static void op_PHY(int operand) {
   if (S >= 0) {
      S = (S - 1) & 255;
   }
   op_STY(operand);
}

static void op_PLA(int operand) {
   A = operand;
   set_NZ(A);
   if (S >= 0) {
      S = (S + 1) & 255;
   }
}

static void op_PLP(int operand) {
   set_NVDIZC(operand);
   if (S >= 0) {
      S = (S + 1) & 255;
   }
}

static void op_PLX(int operand) {
   X = operand;
   set_NZ(X);
   if (S >= 0) {
      S = (S + 1) & 255;
   }
}

static void op_PLY(int operand) {
   Y = operand;
   set_NZ(Y);
   if (S >= 0) {
      S = (S + 1) & 255;
   }
}

static void op_ROLA(int operand) {
   if (A >= 0 && C >= 0) {
      int tmp = (A << 1) + C;
      C = (tmp >> 8) & 1;
      A = tmp & 255;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
}

static void op_ROL(int operand) {
   if (C >= 0) {
      int tmp = (operand << 1) + C;
      C = (tmp >> 8) & 1;
      tmp = tmp & 255;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_RORA(int operand) {
   if (A >= 0 && C >= 0) {
      int tmp = (A >> 1) + (C << 7);
      C = A & 1;
      A = tmp;
      set_NZ(A);
   } else {
      A = -1;
      set_NZC_unknown();
   }
}

static void op_ROR(int operand) {
   if (C >= 0) {
      int tmp = (operand >> 1) + (C << 7);
      C = operand & 1;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_RTS(int operand) {
   if (S >= 0) {
      S = (S + 2) & 255;
   }
}

static void op_RTI(int operand) {
   set_NVDIZC(operand);
   if (S >= 0) {
      S = (S + 3) & 255;
   }
}

static void op_SBC(int operand) {
   // TODO: Decimal mode
   if (A >= 0 && C >= 0) {
      int tmp = A - operand - (1 - C);
      C = 1 - ((tmp >> 8) & 1);
      V = (((A ^ operand) & 0x80) != 0) && (((A ^ tmp) & 0x80) != 0);
      A = tmp & 255;
      set_NZ(A);
   } else {
      A = -1;
      set_NVZC_unknown();
   }
}

static void op_SEC(int operand) {
   C = 1;
}

static void op_SED(int operand) {
   D = 1;
}

static void op_SEI(int operand) {
   I = 1;
}

static void op_STA(int operand) {
   if (A >= 0) {
      if (operand != A) {
         failflag = 1;
      }
   }
   A = operand;
}

static void op_STX(int operand) {
   if (X >= 0) {
      if (operand != X) {
         failflag = 1;
      }
   }
   X = operand;
}

static void op_STY(int operand) {
   if (Y >= 0) {
      if (operand != Y) {
         failflag = 1;
      }
   }
   Y = operand;
}

static void op_TAX(int operand) {
   if (A >= 0) {
      X = A;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_TAY(int operand) {
   if (A >= 0) {
      Y = A;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_TSB_TRB(int operand) {
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_TSX(int operand) {
   if (S >= 0) {
      X = S;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_TXA(int operand) {
   if (X >= 0) {
      A = X;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_TXS(int operand) {
   if (X >= 0) {
      S = X;
   }
}

static void op_TYA(int operand) {
   if (Y >= 0) {
      A = Y;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

void em_init() {
   int i;
   InstrType *instr = instr_table;
   for (i = 0; i < 256; i++) {
      instr->len = addr_mode_table[instr->mode].len;
      instr->fmt = addr_mode_table[instr->mode].fmt;
      instr++;
   }
}

InstrType instr_table[] = {
   { "BRK",  IMM   , 7, WRITEOP,  op_BRK},
   { "ORA",  INDX  , 6, READOP,   op_ORA},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "TSB",  ZP    , 5, READOP,   op_TSB_TRB},
   { "ORA",  ZP    , 3, READOP,   op_ORA},
   { "ASL",  ZP    , 5, READOP,   op_ASL},
   { "RMB0", ZP    , 5, READOP,   0},
   { "PHP",  IMP   , 3, WRITEOP,  op_PHP},
   { "ORA",  IMM   , 2, READOP,   op_ORA},
   { "ASL",  IMPA  , 2, READOP,   op_ASLA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "TSB",  ABS   , 6, READOP,   op_TSB_TRB},
   { "ORA",  ABS   , 4, READOP,   op_ORA},
   { "ASL",  ABS   , 6, READOP,   op_ASL},
   { "BBR0", ZPR   , 5, READOP,   0},
   { "BPL",  BRA   , 2, BRANCHOP, op_BPL},
   { "ORA",  INDY  , 5, READOP,   op_ORA},
   { "ORA",  IND   , 5, READOP,   op_ORA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "TRB",  ZP    , 5, READOP,   op_TSB_TRB},
   { "ORA",  ZPX   , 4, READOP,   op_ORA},
   { "ASL",  ZPX   , 7, READOP,   op_ASL},
   { "RMB1", ZP    , 5, READOP,   0},
   { "CLC",  IMP   , 2, READOP,   op_CLC},
   { "ORA",  ABSY  , 4, READOP,   op_ORA},
   { "INC",  IMPA  , 2, READOP,   op_INCA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "TRB",  ABS   , 6, READOP,   op_TSB_TRB},
   { "ORA",  ABSX  , 4, READOP,   op_ORA},
   { "ASL",  ABSX  , 6, READOP,   op_ASL},
   { "BBR1", ZPR   , 5, READOP,   0},
   { "JSR",  ABS   , 6, READOP,   op_JSR},
   { "AND",  INDX  , 6, READOP,   op_AND},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "BIT",  ZP    , 3, READOP,   op_BIT},
   { "AND",  ZP    , 3, READOP,   op_AND},
   { "ROL",  ZP    , 5, READOP,   op_ROL},
   { "RMB2", ZP    , 5, READOP,   0},
   { "PLP",  IMP   , 4, READOP,   op_PLP},
   { "AND",  IMM   , 2, READOP,   op_AND},
   { "ROL",  IMPA  , 2, READOP,   op_ROLA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "BIT",  ABS   , 4, READOP,   op_BIT},
   { "AND",  ABS   , 4, READOP,   op_AND},
   { "ROL",  ABS   , 6, READOP,   op_ROL},
   { "BBR2", ZPR   , 5, READOP,   0},
   { "BMI",  BRA   , 2, BRANCHOP, op_BMI},
   { "AND",  INDY  , 5, READOP,   op_AND},
   { "AND",  IND   , 5, READOP,   op_AND},
   { "NOP",  IMP   , 1, READOP,   0},
   { "BIT",  ZPX   , 4, READOP,   op_BIT},
   { "AND",  ZPX   , 4, READOP,   op_AND},
   { "ROL",  ZPX   , 6, READOP,   op_ROL},
   { "RMB3", ZP    , 5, READOP,   0},
   { "SEC",  IMP   , 2, READOP,   op_SEC},
   { "AND",  ABSY  , 4, READOP,   op_AND},
   { "DEC",  IMPA  , 2, READOP,   op_DECA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "BIT",  ABSX  , 4, READOP,   op_BIT},
   { "AND",  ABSX  , 4, READOP,   op_AND},
   { "ROL",  ABSX  , 6, READOP,   op_ROL},
   { "BBR3", ZPR   , 5, READOP,   0},
   { "RTI",  IMP   , 6, READOP,   op_RTI},
   { "EOR",  INDX  , 6, READOP,   op_EOR},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 2, READOP,   0},
   { "NOP",  ZP    , 3, READOP,   0},
   { "EOR",  ZP    , 3, READOP,   op_EOR},
   { "LSR",  ZP    , 5, READOP,   op_LSR},
   { "RMB4", ZP    , 5, READOP,   0},
   { "PHA",  IMP   , 3, WRITEOP,  op_PHA},
   { "EOR",  IMM   , 2, READOP,   op_EOR},
   { "LSR",  IMPA  , 2, READOP,   op_LSRA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "JMP",  ABS   , 3, READOP,   0},
   { "EOR",  ABS   , 4, READOP,   op_EOR},
   { "LSR",  ABS   , 6, READOP,   op_LSR},
   { "BBR4", ZPR   , 5, READOP,   0},
   { "BVC",  BRA   , 2, BRANCHOP, op_BVC},
   { "EOR",  INDY  , 5, READOP,   op_EOR},
   { "EOR",  IND   , 5, READOP,   op_EOR},
   { "NOP",  IMP   , 1, READOP,   0},
   { "NOP",  ZPX   , 4, READOP,   0},
   { "EOR",  ZPX   , 4, READOP,   op_EOR},
   { "LSR",  ZPX   , 6, READOP,   op_LSR},
   { "RMB5", ZP    , 5, READOP,   0},
   { "CLI",  IMP   , 2, READOP,   op_CLI},
   { "EOR",  ABSY  , 4, READOP,   op_EOR},
   { "PHY",  IMP   , 3, WRITEOP,  op_PHY},
   { "NOP",  IMP   , 1, READOP,   0},
   { "NOP",  ABS   , 8, READOP,   0},
   { "EOR",  ABSX  , 4, READOP,   op_EOR},
   { "LSR",  ABSX  , 6, READOP,   op_LSR},
   { "BBR5", ZPR   , 5, READOP,   0},
   { "RTS",  IMP   , 6, READOP,   op_RTS},
   { "ADC",  INDX  , 6, READOP,   op_ADC},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STZ",  ZP    , 3, WRITEOP,  0},
   { "ADC",  ZP    , 3, READOP,   op_ADC},
   { "ROR",  ZP    , 5, READOP,   op_ROR},
   { "RMB6", ZP    , 5, READOP,   0},
   { "PLA",  IMP   , 4, READOP,   op_PLA},
   { "ADC",  IMM   , 2, READOP,   op_ADC},
   { "ROR",  IMPA  , 2, READOP,   op_RORA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "JMP",  IND16 , 6, READOP,   0},
   { "ADC",  ABS   , 4, READOP,   op_ADC},
   { "ROR",  ABS   , 6, READOP,   op_ROR},
   { "BBR6", ZPR   , 5, READOP,   0},
   { "BVS",  BRA   , 2, BRANCHOP, op_BVS},
   { "ADC",  INDY  , 5, READOP,   op_ADC},
   { "ADC",  IND   , 5, READOP,   op_ADC},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STZ",  ZPX   , 4, WRITEOP,  0},
   { "ADC",  ZPX   , 4, READOP,   op_ADC},
   { "ROR",  ZPX   , 6, READOP,   op_ROR},
   { "RMB7", ZP    , 5, READOP,   0},
   { "SEI",  IMP   , 2, READOP,   op_SEI},
   { "ADC",  ABSY  , 4, READOP,   op_ADC},
   { "PLY",  IMP   , 4, READOP,   op_PLY},
   { "NOP",  IMP   , 1, READOP,   0},
   { "JMP",  IND1X , 6, READOP,   0},
   { "ADC",  ABSX  , 4, READOP,   op_ADC},
   { "ROR",  ABSX  , 6, READOP,   op_ROR},
   { "BBR7", ZPR   , 5, READOP,   0},
   { "BRA",  BRA   , 3, READOP,   0},
   { "STA",  INDX  , 6, WRITEOP,  op_STA},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STY",  ZP    , 3, WRITEOP,  op_STY},
   { "STA",  ZP    , 3, WRITEOP,  op_STA},
   { "STX",  ZP    , 3, WRITEOP,  op_STX},
   { "SMB0", ZP    , 5, READOP,   0},
   { "DEY",  IMP   , 2, READOP,   op_DEY},
   { "BIT",  IMM   , 2, READOP,   op_BIT_IMM},
   { "TXA",  IMP   , 2, READOP,   op_TXA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STY",  ABS   , 4, WRITEOP,  op_STY},
   { "STA",  ABS   , 4, WRITEOP,  op_STA},
   { "STX",  ABS   , 4, WRITEOP,  op_STX},
   { "BBS0", ZPR   , 5, READOP,   0},
   { "BCC",  BRA   , 2, BRANCHOP, op_BCC},
   { "STA",  INDY  , 6, WRITEOP,  op_STA},
   { "STA",  IND   , 5, WRITEOP,  op_STA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STY",  ZPX   , 4, WRITEOP,  op_STY},
   { "STA",  ZPX   , 4, WRITEOP,  op_STA},
   { "STX",  ZPY   , 4, WRITEOP,  op_STX},
   { "SMB1", ZP    , 5, READOP,   0},
   { "TYA",  IMP   , 2, READOP,   op_TYA},
   { "STA",  ABSY  , 5, WRITEOP,  op_STA},
   { "TXS",  IMP   , 2, READOP,   op_TXS},
   { "NOP",  IMP   , 1, READOP,   0},
   { "STZ",  ABS   , 4, WRITEOP,  0},
   { "STA",  ABSX  , 5, WRITEOP,  op_STA},
   { "STZ",  ABSX  , 5, WRITEOP,  0},
   { "BBS1", ZPR   , 5, READOP,   0},
   { "LDY",  IMM   , 2, READOP,   op_LDY},
   { "LDA",  INDX  , 6, READOP,   op_LDA},
   { "LDX",  IMM   , 2, READOP,   op_LDX},
   { "NOP",  IMP   , 1, READOP,   0},
   { "LDY",  ZP    , 3, READOP,   op_LDY},
   { "LDA",  ZP    , 3, READOP,   op_LDA},
   { "LDX",  ZP    , 3, READOP,   op_LDX},
   { "SMB2", ZP    , 5, READOP,   0},
   { "TAY",  IMP   , 2, READOP,   op_TAY},
   { "LDA",  IMM   , 2, READOP,   op_LDA},
   { "TAX",  IMP   , 2, READOP,   op_TAX},
   { "NOP",  IMP   , 1, READOP,   0},
   { "LDY",  ABS   , 4, READOP,   op_LDY},
   { "LDA",  ABS   , 4, READOP,   op_LDA},
   { "LDX",  ABS   , 4, READOP,   op_LDX},
   { "BBS2", ZPR   , 5, READOP,   0},
   { "BCS",  BRA   , 2, BRANCHOP, op_BCS},
   { "LDA",  INDY  , 5, READOP,   op_LDA},
   { "LDA",  IND   , 5, READOP,   op_LDA},
   { "NOP",  IMP   , 1, READOP,   0},
   { "LDY",  ZPX   , 4, READOP,   op_LDY},
   { "LDA",  ZPX   , 4, READOP,   op_LDA},
   { "LDX",  ZPY   , 4, READOP,   op_LDX},
   { "SMB3", ZP    , 5, READOP,   0},
   { "CLV",  IMP   , 2, READOP,   op_CLV},
   { "LDA",  ABSY  , 4, READOP,   op_LDA},
   { "TSX",  IMP   , 2, READOP,   op_TSX},
   { "NOP",  IMP   , 1, READOP,   0},
   { "LDY",  ABSX  , 4, READOP,   op_LDY},
   { "LDA",  ABSX  , 4, READOP,   op_LDA},
   { "LDX",  ABSY  , 4, READOP,   op_LDX},
   { "BBS3", ZPR   , 5, READOP,   0},
   { "CPY",  IMM   , 2, READOP,   op_CPY},
   { "CMP",  INDX  , 6, READOP,   op_CMP},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "CPY",  ZP    , 3, READOP,   op_CPY},
   { "CMP",  ZP    , 3, READOP,   op_CMP},
   { "DEC",  ZP    , 5, READOP,   op_DEC},
   { "SMB4", ZP    , 5, READOP,   0},
   { "INY",  IMP   , 2, READOP,   op_INY},
   { "CMP",  IMM   , 2, READOP,   op_CMP},
   { "DEX",  IMP   , 2, READOP,   op_DEX},
   { "WAI",  IMP   , 3, READOP,   0},
   { "CPY",  ABS   , 4, READOP,   op_CPY},
   { "CMP",  ABS   , 4, READOP,   op_CMP},
   { "DEC",  ABS   , 6, READOP,   op_DEC},
   { "BBS4", ZPR   , 5, READOP,   0},
   { "BNE",  BRA   , 2, BRANCHOP, op_BNE},
   { "CMP",  INDY  , 5, READOP,   op_CMP},
   { "CMP",  IND   , 5, READOP,   op_CMP},
   { "NOP",  IMP   , 1, READOP,   0},
   { "NOP",  ZPX   , 4, READOP,   0},
   { "CMP",  ZPX   , 4, READOP,   op_CMP},
   { "DEC",  ZPX   , 6, READOP,   op_DEC},
   { "SMB5", ZP    , 5, READOP,   0},
   { "CLD",  IMP   , 2, READOP,   op_CLD},
   { "CMP",  ABSY  , 4, READOP,   op_CMP},
   { "PHX",  IMP   , 3, WRITEOP,  op_PHX},
   { "STP",  IMP   , 3, READOP,   0},
   { "NOP",  ABS   , 4, READOP,   0},
   { "CMP",  ABSX  , 4, READOP,   op_CMP},
   { "DEC",  ABSX  , 7, READOP,   op_DEC},
   { "BBS5", ZPR   , 5, READOP,   0},
   { "CPX",  IMM   , 2, READOP,   op_CPX},
   { "SBC",  INDX  , 6, READOP,   op_SBC},
   { "NOP",  IMM   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "CPX",  ZP    , 3, READOP,   op_CPX},
   { "SBC",  ZP    , 3, READOP,   op_SBC},
   { "INC",  ZP    , 5, READOP,   op_INC},
   { "SMB6", ZP    , 5, READOP,   0},
   { "INX",  IMP   , 2, READOP,   op_INX},
   { "SBC",  IMM   , 2, READOP,   op_SBC},
   { "NOP",  IMP   , 2, READOP,   0},
   { "NOP",  IMP   , 1, READOP,   0},
   { "CPX",  ABS   , 4, READOP,   op_CPX},
   { "SBC",  ABS   , 4, READOP,   op_SBC},
   { "INC",  ABS   , 6, READOP,   op_INC},
   { "BBS6", ZPR   , 5, READOP,   0},
   { "BEQ",  BRA   , 2, BRANCHOP, op_BEQ},
   { "SBC",  INDY  , 5, READOP,   op_SBC},
   { "SBC",  IND   , 5, READOP,   op_SBC},
   { "NOP",  IMP   , 1, READOP,   0},
   { "NOP",  ZPX   , 4, READOP,   0},
   { "SBC",  ZPX   , 4, READOP,   op_SBC},
   { "INC",  ZPX   , 6, READOP,   op_INC},
   { "SMB7", ZP    , 5, READOP,   0},
   { "SED",  IMP   , 2, READOP,   op_SED},
   { "SBC",  ABSY  , 4, READOP,   op_SBC},
   { "PLX",  IMP   , 4, READOP,   op_PLX},
   { "NOP",  IMP   , 1, READOP,   0},
   { "NOP",  ABS   , 4, READOP,   0},
   { "SBC",  ABSX  , 4, READOP,   op_SBC},
   { "INC",  ABSX  , 7, READOP,   op_INC},
   { "BBS7", ZPR   , 5, READOP,   0}
};
