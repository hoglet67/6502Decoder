#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "em_6502.h"

static int c02;

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
   {3,    "%1$s %2$02X,%3$s"}       // ZPR
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

// escaping is to avoid unwanted trigraphs
const char default_fwa[] = "\?\?-\?\?:\?\?\?\?\?\?\?\?:\?\?:\?\? = \?\?\?\?\?\?\?\?\?\?\?\?\?\?\?";

#define OFFSET_SIGN      0
#define OFFSET_EXP       3
#define OFFSET_MANTISSA  6
#define OFFSET_ROUND    15
#define OFFSET_OVERFLOW 18
#define OFFSET_VALUE    23

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


static void op_STA(int operand, int ea);
static void op_STX(int operand, int ea);
static void op_STY(int operand, int ea);

static int memory[0x10000];

static void memory_read(int data, int ea) {
   // TODO: allow memory bounds to be passed in as a command line parameter
   if (ea >= 0 && ea < 0x8000) {
      if (memory[ea] >=0 && memory[ea] != data) {
         printf("memory modelling failed at %04x: expected %02x, actual %02x\n", ea, memory[ea], data);
         failflag |= 1;
      }
      memory[ea] = data;
   }
}

static void memory_write(int data, int ea) {
   if (ea >= 0) {
      if (data < 0 || data > 255) {
         printf("memory modelling failed at %04x: illegal write of %02x\n", ea, data);
         failflag |= 1;
      } else {
         // printf("memory write: %04x = %02x\n", ea, data);
         memory[ea] = data;
      }
   }
}

int compare_NVDIZC(int operand) {
   if (N >= 0) {
      if (N != ((operand >> 7) & 1)) {
         return 1;
      }
   }
   if (V >= 0) {
      if (V != ((operand >> 6) & 1)) {
         return 1;
      }
   }
   if (D >= 0) {
      if (D != ((operand >> 3) & 1)) {
         return 1;
      }
   }
   if (I >= 0) {
      if (I != ((operand >> 2) & 1)) {
         return 1;
      }
   }
   if (Z >= 0) {
      if (Z != ((operand >> 1) & 1)) {
         return 1;
      }
   }
   if (C >= 0) {
      if (C != ((operand >> 0) & 1)) {
         return 1;
      }
   }
   return 0;
}

static void check_NVDIZC(int operand) {
   failflag |= compare_NVDIZC(operand);
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

void em_interrupt(int flags, int pc) {
   if (S >= 0) {
      // Push PCH
      memory_write((pc >> 8) & 255, 0x100 + S);
      S = (S - 1) & 255;
      // Push PCL
      memory_write(pc & 255, 0x100 + S);
      S = (S - 1) & 255;
      // Push P
      memory_write(flags, 0x100 + S);
      S = (S - 1) & 255;
   }
   check_NVDIZC(flags);
   set_NVDIZC(flags);
   I = 1;
   if (c02) {
      D = 0;
   }
}

void em_reset() {
   A = -1;
   X = -1;
   Y = -1;
   S = -1;
   N = -1;
   V = -1;
   D = -1;
   Z = -1;
   C = -1;
   I = 1;
   if (c02) {
      D = 0;
   }
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

int em_get_D() {
   return D;
}

int em_get_I() {
   return I;
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

int em_read_memory(int address) {
   return memory[address];
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
   return buffer;
}

char *em_get_fwa(int a_sign, int a_exp, int a_mantissa, int a_round, int a_overflow) {
   strcpy(buffer, default_fwa);
   int sign     = em_read_memory(a_sign);
   int exp      = em_read_memory(a_exp);
   int man1     = em_read_memory(a_mantissa);
   int man2     = em_read_memory(a_mantissa + 1);
   int man3     = em_read_memory(a_mantissa + 2);
   int man4     = em_read_memory(a_mantissa + 3);
   int round    = em_read_memory(a_round);
   int overflow = a_overflow >= 0 ? em_read_memory(a_overflow) : -1;
   if (sign >= 0) {
      write_hex2(buffer + OFFSET_SIGN, sign);
   }
   if (exp >= 0) {
      write_hex2(buffer + OFFSET_EXP, exp);
   }
   if (man1 >= 0) {
      write_hex2(buffer + OFFSET_MANTISSA + 0, man1);
   }
   if (man2 >= 0) {
      write_hex2(buffer + OFFSET_MANTISSA + 2, man2);
   }
   if (man3 >= 0) {
      write_hex2(buffer + OFFSET_MANTISSA + 4, man3);
   }
   if (man4 >= 0) {
      write_hex2(buffer + OFFSET_MANTISSA + 6, man4);
   }
   if (round >= 0) {
      write_hex2(buffer + OFFSET_ROUND, round);
   }
   if (overflow >= 0) {
      write_hex2(buffer + OFFSET_OVERFLOW, overflow);
   }
   if (sign >= 0 && exp >= 0 && man1 >= 0 && man2 >= 0 && man3 >= 0 && man4 >= 0 && round >= 0) {

      // Real numbers are held in binary floating point format. In the
      // default (40-bit) mode the mantissa is held as a 4 byte binary
      // fraction in sign and magnitude format. Bit 7 of the MSB of
      // the mantissa is the sign bit. When working out the value of
      // the mantissa, this bit is assumed to be 1 (a decimal value of
      // 0.5). The exponent is held as a single byte in 'excess 127'
      // format. In other words, if the actual exponent is zero, the
      // value stored in the exponent byte is 127.

      // Build up a 32 bit mantissa
      uint64_t mantissa = man1;
      mantissa = (mantissa << 8) + man2;
      mantissa = (mantissa << 8) + man3;
      mantissa = (mantissa << 8) + man4;

      // Extend this to 40 bits with the rounding byte
      mantissa = (mantissa << 8) + round;

      // Combine with the exponent
      double value = ((double) mantissa) * pow(2.0, exp - 128 - 40);
      // Take account of the sign
      if (sign & 128) {
         value = -value;
      }
      // Print it to the buffer
      sprintf(buffer + OFFSET_VALUE, "%-+15.8E", value);
   }
   return buffer;
}

int em_get_and_clear_fail() {
   int ret = failflag;
   failflag = 0;
   return ret;
}

static void op_ADC(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode ADC
         int al;
         int ah;
         uint8_t tmp;
         ah = 0;
         Z = N = 0;
         tmp = A + operand + (C ? 1 : 0);
         if (!tmp) {
            Z = 1;
         }
         al = (A & 0xF) + (operand & 0xF) + (C ? 1 : 0);
         if (al > 9) {
            al -= 10;
            al &= 0xF;
            ah = 1;
         }
         ah += ((A >> 4) + (operand >> 4));
         if (ah & 8) {
            N = 1;
         }
         V = (((ah << 4) ^ A) & 128) && !((A ^ operand) & 128);
         C = 0;
         if (ah > 9) {
            C = 1;
            ah -= 10;
            ah &= 0xF;
         }
         A = (al & 0xF) | (ah << 4);
         // On 65C02 ADC, only the NZ flags are different to the 6502
         if (c02) {
            set_NZ(A);
         }
      } else {
         // Normal mode ADC
         int tmp = A + operand + C;
         C = (tmp >> 8) & 1;
         V = (((A ^ operand) & 0x80) == 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
         set_NZ(A);
      }
   } else {
      A = -1;
      set_NVZC_unknown();
   }
}

static void op_AND(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A & operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_ASLA(int operand, int ea) {
   if (A >= 0) {
      C = (A >> 7) & 1;
      A = (A << 1) & 255;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_ASL(int operand, int ea) {
   memory_read(operand, ea);
   C = (operand >> 7) & 1;
   int tmp = (operand << 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_BCC(int branch_taken, int ea) {
   if (C >= 0) {
      if (C == branch_taken) {
         failflag = 1;
      }
   } else {
      C = 1 - branch_taken;
   }
}

static void op_BCS(int branch_taken, int ea) {
   if (C >= 0) {
      if (C != branch_taken) {
         failflag = 1;
      }
   } else {
      C = branch_taken;
   }
}

static void op_BNE(int branch_taken, int ea) {
   if (Z >= 0) {
      if (Z == branch_taken) {
         failflag = 1;
      }
   } else {
      Z = 1 - branch_taken;
   }
}

static void op_BEQ(int branch_taken, int ea) {
   if (Z >= 0) {
      if (Z != branch_taken) {
         failflag = 1;
        }
   } else {
      Z = branch_taken;
   }
}

static void op_BPL(int branch_taken, int ea) {
   if (N >= 0) {
      if (N == branch_taken) {
         failflag = 1;
      }
   } else {
      N = 1 - branch_taken;
   }
}

static void op_BMI(int branch_taken, int ea) {
   if (N >= 0) {
      if (N != branch_taken) {
         failflag = 1;
      }
   } else {
      N = branch_taken;
   }
}

static void op_BVC(int branch_taken, int ea) {
   if (V >= 0) {
      if (V == branch_taken) {
         failflag = 1;
      }
   } else {
      V = 1 - branch_taken;
   }
}

static void op_BVS(int branch_taken, int ea) {
   if (V >= 0) {
      if (V != branch_taken) {
         failflag = 1;
        }
   } else {
      V = branch_taken;
   }
}

static void op_BRK(int operand, int ea) {
   // BRK: the operand is the data pushed to the stack (PCH, PCL, P)
   int flags = operand & 0xff;
   int pc = (operand >> 8) & 0xffff;
   em_interrupt(flags, pc);
}

static void op_BIT_IMM(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_BIT(int operand, int ea) {
   memory_read(operand, ea);
   N = (operand >> 7) & 1;
   V = (operand >> 6) & 1;
   if (A >= 0) {
      Z = (A & operand) == 0;
   } else {
      Z = -1;
   }
}

static void op_CLC(int operand, int ea) {
   C = 0;
}

static void op_CLD(int operand, int ea) {
   D = 0;
}

static void op_CLI(int operand, int ea) {
   I = 0;
}

static void op_CLV(int operand, int ea) {
   V = 0;
}

static void op_CMP(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      int tmp = A - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_CPX(int operand, int ea) {
   memory_read(operand, ea);
   if (X >= 0) {
      int tmp = X - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_CPY(int operand, int ea) {
   memory_read(operand, ea);
   if (Y >= 0) {
      int tmp = Y - operand;
      C = tmp >= 0;
      set_NZ(tmp);
   } else {
      set_NZC_unknown();
   }
}

static void op_DECA(int operand, int ea) {
   if (A >= 0) {
      A = (A - 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEC(int operand, int ea) {
   memory_read(operand, ea);
   int tmp = (operand - 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_DEX(int operand, int ea) {
   if (X >= 0) {
      X = (X - 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_DEY(int operand, int ea) {
   if (Y >= 0) {
      Y = (Y - 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_EOR(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A ^ operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INCA(int operand, int ea) {
   if (A >= 0) {
      A = (A + 1) & 255;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_INC(int operand, int ea) {
   memory_read(operand, ea);
   int tmp = (operand + 1) & 255;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_INX(int operand, int ea) {
   if (X >= 0) {
      X = (X + 1) & 255;
      set_NZ(X);
   } else {
      set_NZ_unknown();
   }
}

static void op_INY(int operand, int ea) {
   if (Y >= 0) {
      Y = (Y + 1) & 255;
      set_NZ(Y);
   } else {
      set_NZ_unknown();
   }
}

static void op_JSR(int operand, int ea) {
   // JSR: the operand is the data pushed to the stack (PCH, PCL)
   if (S >= 0) {
      memory_write((operand >> 8) & 255, 0x100 + S);
      S = (S - 1) & 255;
      memory_write(operand & 255, 0x100 + S);
      S = (S - 1) & 255;
   }
}

static void op_LDA(int operand, int ea) {
   memory_read(operand, ea);
   A = operand;
   set_NZ(A);
}

static void op_LDX(int operand, int ea) {
   memory_read(operand, ea);
   X = operand;
   set_NZ(X);
}

static void op_LDY(int operand, int ea) {
   memory_read(operand, ea);
   Y = operand;
   set_NZ(Y);
}

static void op_LSRA(int operand, int ea) {
   if (A >= 0) {
      C = A & 1;
      A = A >> 1;
      set_NZ(A);
   } else {
      set_NZC_unknown();
   }
}

static void op_LSR(int operand, int ea) {
   memory_read(operand, ea);
   C = operand & 1;
   int tmp = operand >> 1;
   set_NZ(tmp);
   memory_write(tmp, ea);
}

static void op_ORA(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0) {
      A = A | operand;
      set_NZ(A);
   } else {
      set_NZ_unknown();
   }
}

static void op_PHA(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   op_STA(operand, -1);
}

static void op_PHP(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   check_NVDIZC(operand);
   set_NVDIZC(operand);
}

static void op_PHX(int operand, int ea) {
   if (S >= 0) {
      memory_write(operand, 0x100 + S);
      S = (S - 1) & 255;
   }
   op_STX(operand, -1);
}

static void op_PHY(int operand, int ea) {
   if (S >= 0) {
      memory[0x100 + S] = operand;
      S = (S - 1) & 255;
   }
   op_STY(operand, -1);
}

static void op_PLA(int operand, int ea) {
   A = operand;
   set_NZ(A);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLP(int operand, int ea) {
   set_NVDIZC(operand);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLX(int operand, int ea) {
   X = operand;
   set_NZ(X);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_PLY(int operand, int ea) {
   Y = operand;
   set_NZ(Y);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read(operand, 0x100 + S);
   }
}

static void op_ROLA(int operand, int ea) {
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

static void op_ROL(int operand, int ea) {
   if (C >= 0) {
      int tmp = (operand << 1) + C;
      C = (tmp >> 8) & 1;
      tmp = tmp & 255;
      set_NZ(tmp);
      memory_write(tmp, ea);
   } else {
      set_NZC_unknown();
   }
}

static void op_RORA(int operand, int ea) {
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

static void op_ROR(int operand, int ea) {
   if (C >= 0) {
      int tmp = (operand >> 1) + (C << 7);
      C = operand & 1;
      set_NZ(tmp);
      memory_write(tmp, ea);
   } else {
      set_NZC_unknown();
   }
}

static void op_RTS(int operand, int ea) {
   // RTS: the operand is the data pulled from the stack (PCL, PCH)
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);
   }
}

static void op_RTI(int operand, int ea) {
   // RTI: the operand is the data pulled from the stack (P, PCL, PCH)
   set_NVDIZC((operand >> 16) & 255);
   if (S >= 0) {
      S = (S + 1) & 255;
      memory_read((operand >> 16) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read((operand >> 8) & 255, 0x100 + S);
      S = (S + 1) & 255;
      memory_read(operand & 255, 0x100 + S);
   }
}

static void op_SBC(int operand, int ea) {
   memory_read(operand, ea);
   if (A >= 0 && C >= 0) {
      if (D == 1) {
         // Decimal mode SBC
         if (c02) {
            int al;
            int tmp;
            // On 65C02 SBC, both flags and A can be different to the 6502
            al = (A & 15) - (operand & 15) - ((C) ? 0 : 1);
            tmp = A - operand - ((C) ? 0 : 1);
            C = (tmp & 0x100) ? 0 : 1;
            V = ((A ^ operand) & 0x80) && ((A ^ tmp) & 0x80);
            if (tmp < 0) {
               tmp = tmp - 0x60;
            }
            if (al < 0) {
               tmp = tmp - 0x06;
            }
            A = tmp & 255;
            set_NZ(A);
         } else {
            int al;
            int ah;
            int hc = 0;
            uint8_t tmp = A - operand - ((C) ? 0 : 1);
            Z = N = 0;
            if (!(tmp)) {
               Z = 1;
            }
            al = (A & 15) - (operand & 15) - ((C) ? 0 : 1);
            if (al & 16) {
               al -= 6;
               al &= 0xF;
               hc = 1;
            }
            ah = (A >> 4) - (operand >> 4);
            if (hc) {
               ah--;
            }
            if ((A - (operand + ((C) ? 0 : 1))) & 0x80) {
               N = 1;
            }
            V = ((A ^ operand) & 0x80) && ((A ^ tmp) & 0x80);
            C = 1;
            if (ah & 16) {
               C = 0;
               ah -= 6;
               ah &= 0xF;
            }
            A = (al & 0xF) | ((ah & 0xF) << 4);
         }
      } else {
         // Normal mode SBC
         int tmp = A - operand - (1 - C);
         C = 1 - ((tmp >> 8) & 1);
         V = (((A ^ operand) & 0x80) != 0) && (((A ^ tmp) & 0x80) != 0);
         A = tmp & 255;
         set_NZ(A);
      }
   } else {
      A = -1;
      set_NVZC_unknown();
   }
}

static void op_SEC(int operand, int ea) {
   C = 1;
}

static void op_SED(int operand, int ea) {
   D = 1;
}

static void op_SEI(int operand, int ea) {
   I = 1;
}

static void op_STA(int operand, int ea) {
   memory_write(operand, ea);
   if (A >= 0) {
      if (operand != A) {
         failflag = 1;
      }
   }
   A = operand;
}

static void op_STX(int operand, int ea) {
   memory_write(operand, ea);
   if (X >= 0) {
      if (operand != X) {
         failflag = 1;
      }
   }
   X = operand;
}

static void op_STY(int operand, int ea) {
   memory_write(operand, ea);
   if (Y >= 0) {
      if (operand != Y) {
         failflag = 1;
      }
   }
   Y = operand;
}

static void op_STZ(int operand, int ea) {
   memory_write(0, ea);
   if (operand != 0) {
      failflag = 1;
   }
}

static void op_TAX(int operand, int ea) {
   if (A >= 0) {
      X = A;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
}

static void op_TAY(int operand, int ea) {
   if (A >= 0) {
      Y = A;
      set_NZ(Y);
   } else {
      Y = -1;
      set_NZ_unknown();
   }
}

static void op_TSB(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         memory_write(memory[ea] | A, ea);
      }
   } else {
      Z = -1;
   }
}
static void op_TRB(int operand, int ea) {
   if (A >= 0) {
      Z = (A & operand) == 0;
      if (ea >= 0 && memory[ea] >= 0) {
         memory_write(memory[ea] & ~A, ea);
      }
   } else {
      Z = -1;
   }
}

static void op_TSX(int operand, int ea) {
   if (S >= 0) {
      X = S;
      set_NZ(X);
   } else {
      X = -1;
      set_NZ_unknown();
   }
}

static void op_TXA(int operand, int ea) {
   if (X >= 0) {
      A = X;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
}

static void op_TXS(int operand, int ea) {
   if (X >= 0) {
      S = X;
   } else {
      S = -1;
   }
}

static void op_TYA(int operand, int ea) {
   if (Y >= 0) {
      A = Y;
      set_NZ(A);
   } else {
      A = -1;
      set_NZ_unknown();
   }
}

static void op_RMB(int operand, int ea) {
   memory_write(operand, ea);
}

static void op_SMB(int operand, int ea) {
   memory_write(operand, ea);
}


InstrType *instr_table;

static InstrType instr_table_65c02[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, 0, WRITEOP,  op_BRK},
   /* 01 */   { "ORA",  0, INDX  , 6, 0, READOP,   op_ORA},
   /* 02 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* 03 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 04 */   { "TSB",  0, ZP    , 5, 0, TSBTRBOP, op_TSB},
   /* 05 */   { "ORA",  0, ZP    , 3, 0, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, 0, RMWOP,    op_ASL},
   /* 07 */   { "RMB0", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 08 */   { "PHP",  0, IMP   , 3, 0, WRITEOP,  op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, 0, READOP,   op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, 0, READOP,   op_ASLA},
   /* 0B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 0C */   { "TSB",  0, ABS   , 6, 0, TSBTRBOP, op_TSB},
   /* 0D */   { "ORA",  0, ABS   , 4, 0, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, 0, RMWOP,    op_ASL},
   /* 0F */   { "BBR0", 0, ZPR   , 5, 0, READOP,   0},
   /* 10 */   { "BPL",  0, BRA   , 2, 0, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, 0, READOP,   op_ORA},
   /* 12 */   { "ORA",  0, IND   , 5, 0, READOP,   op_ORA},
   /* 13 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 14 */   { "TRB",  0, ZP    , 5, 0, TSBTRBOP, op_TRB},
   /* 15 */   { "ORA",  0, ZPX   , 4, 0, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, 0, RMWOP,    op_ASL},
   /* 17 */   { "RMB1", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 18 */   { "CLC",  0, IMP   , 2, 0, READOP,   op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, 0, READOP,   op_ORA},
   /* 1A */   { "INC",  0, IMPA  , 2, 0, READOP,   op_INCA},
   /* 1B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 1C */   { "TRB",  0, ABS   , 6, 0, TSBTRBOP, op_TRB},
   /* 1D */   { "ORA",  0, ABSX  , 4, 0, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 6, 0, RMWOP,    op_ASL},
   /* 1F */   { "BBR1", 0, ZPR   , 5, 0, READOP,   0},
   /* 20 */   { "JSR",  0, ABS   , 6, 0, READOP,   op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, 0, READOP,   op_AND},
   /* 22 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* 23 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 24 */   { "BIT",  0, ZP    , 3, 0, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, 0, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, 0, RMWOP,    op_ROL},
   /* 27 */   { "RMB2", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 28 */   { "PLP",  0, IMP   , 4, 0, READOP,   op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, 0, READOP,   op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, 0, READOP,   op_ROLA},
   /* 2B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 2C */   { "BIT",  0, ABS   , 4, 0, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, 0, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, 0, RMWOP,    op_ROL},
   /* 2F */   { "BBR2", 0, ZPR   , 5, 0, READOP,   0},
   /* 30 */   { "BMI",  0, BRA   , 2, 0, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, 0, READOP,   op_AND},
   /* 32 */   { "AND",  0, IND   , 5, 0, READOP,   op_AND},
   /* 33 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 34 */   { "BIT",  0, ZPX   , 4, 0, READOP,   op_BIT},
   /* 35 */   { "AND",  0, ZPX   , 4, 0, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, 0, RMWOP,    op_ROL},
   /* 37 */   { "RMB3", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 38 */   { "SEC",  0, IMP   , 2, 0, READOP,   op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, 0, READOP,   op_AND},
   /* 3A */   { "DEC",  0, IMPA  , 2, 0, READOP,   op_DECA},
   /* 3B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 3C */   { "BIT",  0, ABSX  , 4, 0, READOP,   op_BIT},
   /* 3D */   { "AND",  0, ABSX  , 4, 0, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 6, 0, RMWOP,    op_ROL},
   /* 3F */   { "BBR3", 0, ZPR   , 5, 0, READOP,   0},
   /* 40 */   { "RTI",  0, IMP   , 6, 0, READOP,   op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, 0, READOP,   op_EOR},
   /* 42 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* 43 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 44 */   { "NOP",  0, ZP    , 3, 0, READOP,   0},
   /* 45 */   { "EOR",  0, ZP    , 3, 0, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, 0, RMWOP,    op_LSR},
   /* 47 */   { "RMB4", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 48 */   { "PHA",  0, IMP   , 3, 0, WRITEOP,  op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, 0, READOP,   op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, 0, READOP,   op_LSRA},
   /* 4B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 4C */   { "JMP",  0, ABS   , 3, 0, READOP,   0},
   /* 4D */   { "EOR",  0, ABS   , 4, 0, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, 0, RMWOP,    op_LSR},
   /* 4F */   { "BBR4", 0, ZPR   , 5, 0, READOP,   0},
   /* 50 */   { "BVC",  0, BRA   , 2, 0, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, 0, READOP,   op_EOR},
   /* 52 */   { "EOR",  0, IND   , 5, 0, READOP,   op_EOR},
   /* 53 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 54 */   { "NOP",  0, ZPX   , 4, 0, READOP,   0},
   /* 55 */   { "EOR",  0, ZPX   , 4, 0, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, 0, RMWOP,    op_LSR},
   /* 57 */   { "RMB5", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 58 */   { "CLI",  0, IMP   , 2, 0, READOP,   op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, 0, READOP,   op_EOR},
   /* 5A */   { "PHY",  0, IMP   , 3, 0, WRITEOP,  op_PHY},
   /* 5B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 5C */   { "NOP",  0, ABS   , 8, 0, READOP,   0},
   /* 5D */   { "EOR",  0, ABSX  , 4, 0, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 6, 0, RMWOP,    op_LSR},
   /* 5F */   { "BBR5", 0, ZPR   , 5, 0, READOP,   0},
   /* 60 */   { "RTS",  0, IMP   , 6, 0, READOP,   op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, 1, READOP,   op_ADC},
   /* 62 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* 63 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 64 */   { "STZ",  0, ZP    , 3, 0, WRITEOP,  op_STZ},
   /* 65 */   { "ADC",  0, ZP    , 3, 1, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, 0, RMWOP,    op_ROR},
   /* 67 */   { "RMB6", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 68 */   { "PLA",  0, IMP   , 4, 0, READOP,   op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, 1, READOP,   op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, 0, READOP,   op_RORA},
   /* 6B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 6C */   { "JMP",  0, IND16 , 6, 0, READOP,   0},
   /* 6D */   { "ADC",  0, ABS   , 4, 1, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, 0, RMWOP,    op_ROR},
   /* 6F */   { "BBR6", 0, ZPR   , 5, 0, READOP,   0},
   /* 70 */   { "BVS",  0, BRA   , 2, 0, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, 1, READOP,   op_ADC},
   /* 72 */   { "ADC",  0, IND   , 5, 1, READOP,   op_ADC},
   /* 73 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 74 */   { "STZ",  0, ZPX   , 4, 0, WRITEOP,  op_STZ},
   /* 75 */   { "ADC",  0, ZPX   , 4, 1, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, 0, RMWOP,    op_ROR},
   /* 77 */   { "RMB7", 0, ZP    , 5, 0, READOP,   op_RMB},
   /* 78 */   { "SEI",  0, IMP   , 2, 0, READOP,   op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, 1, READOP,   op_ADC},
   /* 7A */   { "PLY",  0, IMP   , 4, 0, READOP,   op_PLY},
   /* 7B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 7C */   { "JMP",  0, IND1X , 6, 0, READOP,   0},
   /* 7D */   { "ADC",  0, ABSX  , 4, 1, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 6, 0, RMWOP,    op_ROR},
   /* 7F */   { "BBR7", 0, ZPR   , 5, 0, READOP,   0},
   /* 80 */   { "BRA",  0, BRA   , 3, 0, READOP,   0},
   /* 81 */   { "STA",  0, INDX  , 6, 0, WRITEOP,  op_STA},
   /* 82 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* 83 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 84 */   { "STY",  0, ZP    , 3, 0, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, 0, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, 0, WRITEOP,  op_STX},
   /* 87 */   { "SMB0", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* 88 */   { "DEY",  0, IMP   , 2, 0, READOP,   op_DEY},
   /* 89 */   { "BIT",  0, IMM   , 2, 0, READOP,   op_BIT_IMM},
   /* 8A */   { "TXA",  0, IMP   , 2, 0, READOP,   op_TXA},
   /* 8B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 8C */   { "STY",  0, ABS   , 4, 0, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, 0, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, 0, WRITEOP,  op_STX},
   /* 8F */   { "BBS0", 0, ZPR   , 5, 0, READOP,   0},
   /* 90 */   { "BCC",  0, BRA   , 2, 0, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, 0, WRITEOP,  op_STA},
   /* 92 */   { "STA",  0, IND   , 5, 0, WRITEOP,  op_STA},
   /* 93 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 94 */   { "STY",  0, ZPX   , 4, 0, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, 0, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, 0, WRITEOP,  op_STX},
   /* 97 */   { "SMB1", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* 98 */   { "TYA",  0, IMP   , 2, 0, READOP,   op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, 0, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, 0, READOP,   op_TXS},
   /* 9B */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* 9C */   { "STZ",  0, ABS   , 4, 0, WRITEOP,  op_STZ},
   /* 9D */   { "STA",  0, ABSX  , 5, 0, WRITEOP,  op_STA},
   /* 9E */   { "STZ",  0, ABSX  , 5, 0, WRITEOP,  op_STZ},
   /* 9F */   { "BBS1", 0, ZPR   , 5, 0, READOP,   0},
   /* A0 */   { "LDY",  0, IMM   , 2, 0, READOP,   op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, 0, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, 0, READOP,   op_LDX},
   /* A3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* A4 */   { "LDY",  0, ZP    , 3, 0, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, 0, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, 0, READOP,   op_LDX},
   /* A7 */   { "SMB2", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* A8 */   { "TAY",  0, IMP   , 2, 0, READOP,   op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, 0, READOP,   op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, 0, READOP,   op_TAX},
   /* AB */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* AC */   { "LDY",  0, ABS   , 4, 0, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, 0, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, 0, READOP,   op_LDX},
   /* AF */   { "BBS2", 0, ZPR   , 5, 0, READOP,   0},
   /* B0 */   { "BCS",  0, BRA   , 2, 0, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, 0, READOP,   op_LDA},
   /* B2 */   { "LDA",  0, IND   , 5, 0, READOP,   op_LDA},
   /* B3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* B4 */   { "LDY",  0, ZPX   , 4, 0, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, 0, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, 0, READOP,   op_LDX},
   /* B7 */   { "SMB3", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* B8 */   { "CLV",  0, IMP   , 2, 0, READOP,   op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, 0, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, 0, READOP,   op_TSX},
   /* BB */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* BC */   { "LDY",  0, ABSX  , 4, 0, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, 0, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, 0, READOP,   op_LDX},
   /* BF */   { "BBS3", 0, ZPR   , 5, 0, READOP,   0},
   /* C0 */   { "CPY",  0, IMM   , 2, 0, READOP,   op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, 0, READOP,   op_CMP},
   /* C2 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* C3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* C4 */   { "CPY",  0, ZP    , 3, 0, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, 0, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, 0, RMWOP,    op_DEC},
   /* C7 */   { "SMB4", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* C8 */   { "INY",  0, IMP   , 2, 0, READOP,   op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, 0, READOP,   op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, 0, READOP,   op_DEX},
   /* CB */   { "WAI",  0, IMP   , 1, 0, READOP,   0},        // WD65C02=3
   /* CC */   { "CPY",  0, ABS   , 4, 0, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, 0, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, 0, RMWOP,    op_DEC},
   /* CF */   { "BBS4", 0, ZPR   , 5, 0, READOP,   0},
   /* D0 */   { "BNE",  0, BRA   , 2, 0, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, 0, READOP,   op_CMP},
   /* D2 */   { "CMP",  0, IND   , 5, 0, READOP,   op_CMP},
   /* D3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* D4 */   { "NOP",  0, ZPX   , 4, 0, READOP,   0},
   /* D5 */   { "CMP",  0, ZPX   , 4, 0, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, 0, RMWOP,    op_DEC},
   /* D7 */   { "SMB5", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* D8 */   { "CLD",  0, IMP   , 2, 0, READOP,   op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, 0, READOP,   op_CMP},
   /* DA */   { "PHX",  0, IMP   , 3, 0, WRITEOP,  op_PHX},
   /* DB */   { "STP",  0, IMP   , 1, 0, READOP,   0},        // WD65C02=3
   /* DC */   { "NOP",  0, ABS   , 4, 0, READOP,   0},
   /* DD */   { "CMP",  0, ABSX  , 4, 0, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, 0, RMWOP,    op_DEC},
   /* DF */   { "BBS5", 0, ZPR   , 5, 0, READOP,   0},
   /* E0 */   { "CPX",  0, IMM   , 2, 0, READOP,   op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, 1, READOP,   op_SBC},
   /* E2 */   { "NOP",  0, IMM   , 2, 0, READOP,   0},
   /* E3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* E4 */   { "CPX",  0, ZP    , 3, 0, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, 1, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, 0, RMWOP,    op_INC},
   /* E7 */   { "SMB6", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* E8 */   { "INX",  0, IMP   , 2, 0, READOP,   op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, 1, READOP,   op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, 0, READOP,   0},
   /* EB */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* EC */   { "CPX",  0, ABS   , 4, 0, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, 1, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, 0, RMWOP,    op_INC},
   /* EF */   { "BBS6", 0, ZPR   , 5, 0, READOP,   0},
   /* F0 */   { "BEQ",  0, BRA   , 2, 0, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, 1, READOP,   op_SBC},
   /* F2 */   { "SBC",  0, IND   , 5, 1, READOP,   op_SBC},
   /* F3 */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* F4 */   { "NOP",  0, ZPX   , 4, 0, READOP,   0},
   /* F5 */   { "SBC",  0, ZPX   , 4, 1, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, 0, RMWOP,    op_INC},
   /* F7 */   { "SMB7", 0, ZP    , 5, 0, READOP,   op_SMB},
   /* F8 */   { "SED",  0, IMP   , 2, 0, READOP,   op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, 1, READOP,   op_SBC},
   /* FA */   { "PLX",  0, IMP   , 4, 0, READOP,   op_PLX},
   /* FB */   { "NOP",  0, IMP   , 1, 0, READOP,   0},
   /* FC */   { "NOP",  0, ABS   , 4, 0, READOP,   0},
   /* FD */   { "SBC",  0, ABSX  , 4, 1, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, 0, RMWOP,    op_INC},
   /* FF */   { "BBS7", 0, ZPR   , 5, 0, READOP,   0}
};

static InstrType instr_table_6502[] = {
   /* 00 */   { "BRK",  0, IMM   , 7, 0, WRITEOP,  op_BRK},
   /* 01 */   { "ORA",  0, INDX  , 6, 0, READOP,   op_ORA},
   /* 02 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 03 */   { "SLO",  1, INDX  , 8, 0, READOP,   0},
   /* 04 */   { "NOP",  1, ZP    , 3, 0, READOP,   0},
   /* 05 */   { "ORA",  0, ZP    , 3, 0, READOP,   op_ORA},
   /* 06 */   { "ASL",  0, ZP    , 5, 0, RMWOP,    op_ASL},
   /* 07 */   { "SLO",  1, ZP    , 5, 0, READOP,   0},
   /* 08 */   { "PHP",  0, IMP   , 3, 0, WRITEOP,  op_PHP},
   /* 09 */   { "ORA",  0, IMM   , 2, 0, READOP,   op_ORA},
   /* 0A */   { "ASL",  0, IMPA  , 2, 0, READOP,   op_ASLA},
   /* 0B */   { "ANC",  1, IMM   , 2, 0, READOP,   0},
   /* 0C */   { "NOP",  1, ABS   , 4, 0, READOP,   0},
   /* 0D */   { "ORA",  0, ABS   , 4, 0, READOP,   op_ORA},
   /* 0E */   { "ASL",  0, ABS   , 6, 0, RMWOP,    op_ASL},
   /* 0F */   { "SLO",  1, ABS   , 6, 0, READOP,   0},
   /* 10 */   { "BPL",  0, BRA   , 2, 0, BRANCHOP, op_BPL},
   /* 11 */   { "ORA",  0, INDY  , 5, 0, READOP,   op_ORA},
   /* 12 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 13 */   { "SLO",  1, INDY  , 8, 0, READOP,   0},
   /* 14 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* 15 */   { "ORA",  0, ZPX   , 4, 0, READOP,   op_ORA},
   /* 16 */   { "ASL",  0, ZPX   , 6, 0, RMWOP,    op_ASL},
   /* 17 */   { "SLO",  1, ZPX   , 6, 0, READOP,   0},
   /* 18 */   { "CLC",  0, IMP   , 2, 0, READOP,   op_CLC},
   /* 19 */   { "ORA",  0, ABSY  , 4, 0, READOP,   op_ORA},
   /* 1A */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* 1B */   { "SLO",  1, ABSY  , 7, 0, READOP,   0},
   /* 1C */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* 1D */   { "ORA",  0, ABSX  , 4, 0, READOP,   op_ORA},
   /* 1E */   { "ASL",  0, ABSX  , 7, 0, RMWOP,    op_ASL},
   /* 1F */   { "SLO",  1, ABSX  , 4, 0, READOP,   0},
   /* 20 */   { "JSR",  0, ABS   , 6, 0, READOP,   op_JSR},
   /* 21 */   { "AND",  0, INDX  , 6, 0, READOP,   op_AND},
   /* 22 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 23 */   { "RLA",  1, INDX  , 8, 0, READOP,   0},
   /* 24 */   { "BIT",  0, ZP    , 3, 0, READOP,   op_BIT},
   /* 25 */   { "AND",  0, ZP    , 3, 0, READOP,   op_AND},
   /* 26 */   { "ROL",  0, ZP    , 5, 0, RMWOP,    op_ROL},
   /* 27 */   { "RLA",  1, ZP    , 5, 0, READOP,   0},
   /* 28 */   { "PLP",  0, IMP   , 4, 0, READOP,   op_PLP},
   /* 29 */   { "AND",  0, IMM   , 2, 0, READOP,   op_AND},
   /* 2A */   { "ROL",  0, IMPA  , 2, 0, READOP,   op_ROLA},
   /* 2B */   { "ANC",  1, IMM   , 2, 0, READOP,   0},
   /* 2C */   { "BIT",  0, ABS   , 4, 0, READOP,   op_BIT},
   /* 2D */   { "AND",  0, ABS   , 4, 0, READOP,   op_AND},
   /* 2E */   { "ROL",  0, ABS   , 6, 0, RMWOP,    op_ROL},
   /* 2F */   { "RLA",  1, ABS   , 6, 0, READOP,   0},
   /* 30 */   { "BMI",  0, BRA   , 2, 0, BRANCHOP, op_BMI},
   /* 31 */   { "AND",  0, INDY  , 5, 0, READOP,   op_AND},
   /* 32 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 33 */   { "RLA",  1, INDY  , 8, 0, READOP,   0},
   /* 34 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* 35 */   { "AND",  0, ZPX   , 4, 0, READOP,   op_AND},
   /* 36 */   { "ROL",  0, ZPX   , 6, 0, RMWOP,    op_ROL},
   /* 37 */   { "RLA",  1, ZPX   , 6, 0, READOP,   0},
   /* 38 */   { "SEC",  0, IMP   , 2, 0, READOP,   op_SEC},
   /* 39 */   { "AND",  0, ABSY  , 4, 0, READOP,   op_AND},
   /* 3A */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* 3B */   { "RLA",  1, ABSY  , 7, 0, READOP,   0},
   /* 3C */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* 3D */   { "AND",  0, ABSX  , 4, 0, READOP,   op_AND},
   /* 3E */   { "ROL",  0, ABSX  , 7, 0, RMWOP,    op_ROL},
   /* 3F */   { "RLA",  1, ABSX  , 7, 0, READOP,   0},
   /* 40 */   { "RTI",  0, IMP   , 6, 0, READOP,   op_RTI},
   /* 41 */   { "EOR",  0, INDX  , 6, 0, READOP,   op_EOR},
   /* 42 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 43 */   { "SRE",  1, INDX  , 8, 0, READOP,   0},
   /* 44 */   { "NOP",  1, ZP    , 3, 0, READOP,   0},
   /* 45 */   { "EOR",  0, ZP    , 3, 0, READOP,   op_EOR},
   /* 46 */   { "LSR",  0, ZP    , 5, 0, RMWOP,    op_LSR},
   /* 47 */   { "SRE",  1, ZP    , 5, 0, READOP,   0},
   /* 48 */   { "PHA",  0, IMP   , 3, 0, WRITEOP,  op_PHA},
   /* 49 */   { "EOR",  0, IMM   , 2, 0, READOP,   op_EOR},
   /* 4A */   { "LSR",  0, IMPA  , 2, 0, READOP,   op_LSRA},
   /* 4B */   { "ALR",  1, IMM   , 2, 0, READOP,   0},
   /* 4C */   { "JMP",  0, ABS   , 3, 0, READOP,   0},
   /* 4D */   { "EOR",  0, ABS   , 4, 0, READOP,   op_EOR},
   /* 4E */   { "LSR",  0, ABS   , 6, 0, RMWOP,    op_LSR},
   /* 4F */   { "SRE",  1, ABS   , 6, 0, READOP,   0},
   /* 50 */   { "BVC",  0, BRA   , 2, 0, BRANCHOP, op_BVC},
   /* 51 */   { "EOR",  0, INDY  , 5, 0, READOP,   op_EOR},
   /* 52 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 53 */   { "SRE",  1, INDY  , 8, 0, READOP,   0},
   /* 54 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* 55 */   { "EOR",  0, ZPX   , 4, 0, READOP,   op_EOR},
   /* 56 */   { "LSR",  0, ZPX   , 6, 0, RMWOP,    op_LSR},
   /* 57 */   { "SRE",  1, ZPX   , 6, 0, READOP,   0},
   /* 58 */   { "CLI",  0, IMP   , 2, 0, READOP,   op_CLI},
   /* 59 */   { "EOR",  0, ABSY  , 4, 0, READOP,   op_EOR},
   /* 5A */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* 5B */   { "SRE",  1, ABSY  , 7, 0, READOP,   0},
   /* 5C */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* 5D */   { "EOR",  0, ABSX  , 4, 0, READOP,   op_EOR},
   /* 5E */   { "LSR",  0, ABSX  , 7, 0, RMWOP,    op_LSR},
   /* 5F */   { "SRX",  1, ABSX  , 7, 0, READOP,   0},
   /* 60 */   { "RTS",  0, IMP   , 6, 0, READOP,   op_RTS},
   /* 61 */   { "ADC",  0, INDX  , 6, 0, READOP,   op_ADC},
   /* 62 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 63 */   { "RRA",  1, INDX  , 8, 0, READOP,   0},
   /* 64 */   { "NOP",  1, ZP    , 3, 0, READOP,   0},
   /* 65 */   { "ADC",  0, ZP    , 3, 0, READOP,   op_ADC},
   /* 66 */   { "ROR",  0, ZP    , 5, 0, RMWOP,    op_ROR},
   /* 67 */   { "RRA",  1, ZP    , 5, 0, READOP,   0},
   /* 68 */   { "PLA",  0, IMP   , 4, 0, READOP,   op_PLA},
   /* 69 */   { "ADC",  0, IMM   , 2, 0, READOP,   op_ADC},
   /* 6A */   { "ROR",  0, IMPA  , 2, 0, READOP,   op_RORA},
   /* 6B */   { "ARR",  1, IMM   , 2, 0, READOP,   0},
   /* 6C */   { "JMP",  0, IND16 , 5, 0, READOP,   0},
   /* 6D */   { "ADC",  0, ABS   , 4, 0, READOP,   op_ADC},
   /* 6E */   { "ROR",  0, ABS   , 6, 0, RMWOP,    op_ROR},
   /* 6F */   { "RRA",  1, ABS   , 6, 0, READOP,   0},
   /* 70 */   { "BVS",  0, BRA   , 2, 0, BRANCHOP, op_BVS},
   /* 71 */   { "ADC",  0, INDY  , 5, 0, READOP,   op_ADC},
   /* 72 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 73 */   { "RRA",  1, INDY  , 8, 0, READOP,   0},
   /* 74 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* 75 */   { "ADC",  0, ZPX   , 4, 0, READOP,   op_ADC},
   /* 76 */   { "ROR",  0, ZPX   , 6, 0, RMWOP,    op_ROR},
   /* 77 */   { "RRA",  1, ZPX   , 6, 0, READOP,   0},
   /* 78 */   { "SEI",  0, IMP   , 2, 0, READOP,   op_SEI},
   /* 79 */   { "ADC",  0, ABSY  , 4, 0, READOP,   op_ADC},
   /* 7A */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* 7B */   { "RRA",  1, ABSY  , 8, 0, READOP,   0},
   /* 7C */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* 7D */   { "ADC",  0, ABSX  , 4, 0, READOP,   op_ADC},
   /* 7E */   { "ROR",  0, ABSX  , 7, 0, RMWOP,    op_ROR},
   /* 7F */   { "RRA",  1, ABSX  , 8, 0, READOP,   0},
   /* 80 */   { "NOP",  1, IMM   , 2, 0, READOP,   0},
   /* 81 */   { "STA",  0, INDX  , 6, 0, WRITEOP,  op_STA},
   /* 82 */   { "NOP",  1, IMM   , 2, 0, READOP,   0},
   /* 83 */   { "SAX",  1, INDX  , 6, 0, READOP,   0},
   /* 84 */   { "STY",  0, ZP    , 3, 0, WRITEOP,  op_STY},
   /* 85 */   { "STA",  0, ZP    , 3, 0, WRITEOP,  op_STA},
   /* 86 */   { "STX",  0, ZP    , 3, 0, WRITEOP,  op_STX},
   /* 87 */   { "SAX",  1, ZP    , 3, 0, READOP,   0},
   /* 88 */   { "DEY",  0, IMP   , 2, 0, READOP,   op_DEY},
   /* 89 */   { "NOP",  1, IMM   , 2, 0, READOP,   0},
   /* 8A */   { "TXA",  0, IMP   , 2, 0, READOP,   op_TXA},
   /* 8B */   { "XXA",  1, IMM   , 2, 0, READOP,   0},
   /* 8C */   { "STY",  0, ABS   , 4, 0, WRITEOP,  op_STY},
   /* 8D */   { "STA",  0, ABS   , 4, 0, WRITEOP,  op_STA},
   /* 8E */   { "STX",  0, ABS   , 4, 0, WRITEOP,  op_STX},
   /* 8F */   { "SAX",  1, ABS   , 4, 0, READOP,   0},
   /* 90 */   { "BCC",  0, BRA   , 2, 0, BRANCHOP, op_BCC},
   /* 91 */   { "STA",  0, INDY  , 6, 0, WRITEOP,  op_STA},
   /* 92 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* 93 */   { "AHX",  1, INDY  , 6, 0, READOP,   0},
   /* 94 */   { "STY",  0, ZPX   , 4, 0, WRITEOP,  op_STY},
   /* 95 */   { "STA",  0, ZPX   , 4, 0, WRITEOP,  op_STA},
   /* 96 */   { "STX",  0, ZPY   , 4, 0, WRITEOP,  op_STX},
   /* 97 */   { "SAX",  1, ZPY   , 4, 0, READOP,   0},
   /* 98 */   { "TYA",  0, IMP   , 2, 0, READOP,   op_TYA},
   /* 99 */   { "STA",  0, ABSY  , 5, 0, WRITEOP,  op_STA},
   /* 9A */   { "TXS",  0, IMP   , 2, 0, READOP,   op_TXS},
   /* 9B */   { "TAS",  1, ABS   , 5, 0, READOP,   0},
   /* 9C */   { "SHY",  1, ABSX  , 5, 0, READOP,   0},
   /* 9D */   { "STA",  0, ABSX  , 5, 0, WRITEOP,  op_STA},
   /* 9E */   { "SHX",  1, ABSY  , 5, 0, READOP,   0},
   /* 9F */   { "AHX",  1, ABSY  , 5, 0, READOP,   0},
   /* A0 */   { "LDY",  0, IMM   , 2, 0, READOP,   op_LDY},
   /* A1 */   { "LDA",  0, INDX  , 6, 0, READOP,   op_LDA},
   /* A2 */   { "LDX",  0, IMM   , 2, 0, READOP,   op_LDX},
   /* A3 */   { "LAX",  1, INDX  , 6, 0, READOP,   0},
   /* A4 */   { "LDY",  0, ZP    , 3, 0, READOP,   op_LDY},
   /* A5 */   { "LDA",  0, ZP    , 3, 0, READOP,   op_LDA},
   /* A6 */   { "LDX",  0, ZP    , 3, 0, READOP,   op_LDX},
   /* A7 */   { "LAX",  1, ZP    , 3, 0, READOP,   0},
   /* A8 */   { "TAY",  0, IMP   , 2, 0, READOP,   op_TAY},
   /* A9 */   { "LDA",  0, IMM   , 2, 0, READOP,   op_LDA},
   /* AA */   { "TAX",  0, IMP   , 2, 0, READOP,   op_TAX},
   /* AB */   { "LAX",  1, IMM   , 2, 0, READOP,   0},
   /* AC */   { "LDY",  0, ABS   , 4, 0, READOP,   op_LDY},
   /* AD */   { "LDA",  0, ABS   , 4, 0, READOP,   op_LDA},
   /* AE */   { "LDX",  0, ABS   , 4, 0, READOP,   op_LDX},
   /* AF */   { "LAX",  1, ABS   , 4, 0, READOP,   0},
   /* B0 */   { "BCS",  0, BRA   , 2, 0, BRANCHOP, op_BCS},
   /* B1 */   { "LDA",  0, INDY  , 5, 0, READOP,   op_LDA},
   /* B2 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* B3 */   { "LAX",  1, INDY  , 5, 0, READOP,   0},
   /* B4 */   { "LDY",  0, ZPX   , 4, 0, READOP,   op_LDY},
   /* B5 */   { "LDA",  0, ZPX   , 4, 0, READOP,   op_LDA},
   /* B6 */   { "LDX",  0, ZPY   , 4, 0, READOP,   op_LDX},
   /* B7 */   { "LAX",  1, ZPY   , 4, 0, READOP,   0},
   /* B8 */   { "CLV",  0, IMP   , 2, 0, READOP,   op_CLV},
   /* B9 */   { "LDA",  0, ABSY  , 4, 0, READOP,   op_LDA},
   /* BA */   { "TSX",  0, IMP   , 2, 0, READOP,   op_TSX},
   /* BB */   { "LAS",  1, ABSY  , 4, 0, READOP,   0},
   /* BC */   { "LDY",  0, ABSX  , 4, 0, READOP,   op_LDY},
   /* BD */   { "LDA",  0, ABSX  , 4, 0, READOP,   op_LDA},
   /* BE */   { "LDX",  0, ABSY  , 4, 0, READOP,   op_LDX},
   /* BF */   { "LAX",  1, ABSY  , 4, 0, READOP,   0},
   /* C0 */   { "CPY",  0, IMM   , 2, 0, READOP,   op_CPY},
   /* C1 */   { "CMP",  0, INDX  , 6, 0, READOP,   op_CMP},
   /* C2 */   { "NOP",  1, IMP   , 0, 0, READOP,   0},
   /* C3 */   { "DCP",  1, INDX  , 8, 0, READOP,   0},
   /* C4 */   { "CPY",  0, ZP    , 3, 0, READOP,   op_CPY},
   /* C5 */   { "CMP",  0, ZP    , 3, 0, READOP,   op_CMP},
   /* C6 */   { "DEC",  0, ZP    , 5, 0, RMWOP,    op_DEC},
   /* C7 */   { "DCP",  1, ZP    , 5, 0, READOP,   0},
   /* C8 */   { "INY",  0, IMP   , 2, 0, READOP,   op_INY},
   /* C9 */   { "CMP",  0, IMM   , 2, 0, READOP,   op_CMP},
   /* CA */   { "DEX",  0, IMP   , 2, 0, READOP,   op_DEX},
   /* CB */   { "AXS",  1, IMM   , 2, 0, READOP,   0},
   /* CC */   { "CPY",  0, ABS   , 4, 0, READOP,   op_CPY},
   /* CD */   { "CMP",  0, ABS   , 4, 0, READOP,   op_CMP},
   /* CE */   { "DEC",  0, ABS   , 6, 0, RMWOP,    op_DEC},
   /* CF */   { "DCP",  1, ABS   , 6, 0, READOP,   0},
   /* D0 */   { "BNE",  0, BRA   , 2, 0, BRANCHOP, op_BNE},
   /* D1 */   { "CMP",  0, INDY  , 5, 0, READOP,   op_CMP},
   /* D2 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* D3 */   { "DCP",  1, INDY  , 8, 0, READOP,   0},
   /* D4 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* D5 */   { "CMP",  0, ZPX   , 4, 0, READOP,   op_CMP},
   /* D6 */   { "DEC",  0, ZPX   , 6, 0, RMWOP,    op_DEC},
   /* D7 */   { "DCP",  1, ZPX   , 6, 0, READOP,   0},
   /* D8 */   { "CLD",  0, IMP   , 2, 0, READOP,   op_CLD},
   /* D9 */   { "CMP",  0, ABSY  , 4, 0, READOP,   op_CMP},
   /* DA */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* DB */   { "DCP",  1, ABSY  , 7, 0, READOP,   0},
   /* DC */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* DD */   { "CMP",  0, ABSX  , 4, 0, READOP,   op_CMP},
   /* DE */   { "DEC",  0, ABSX  , 7, 0, RMWOP,    op_DEC},
   /* DF */   { "DCP",  1, ABSX  , 7, 0, READOP,   0},
   /* E0 */   { "CPX",  0, IMM   , 2, 0, READOP,   op_CPX},
   /* E1 */   { "SBC",  0, INDX  , 6, 0, READOP,   op_SBC},
   /* E2 */   { "NOP",  1, IMP   , 0, 0, READOP,   0},
   /* E3 */   { "ISC",  1, INDX  , 8, 0, READOP,   0},
   /* E4 */   { "CPX",  0, ZP    , 3, 0, READOP,   op_CPX},
   /* E5 */   { "SBC",  0, ZP    , 3, 0, READOP,   op_SBC},
   /* E6 */   { "INC",  0, ZP    , 5, 0, RMWOP,    op_INC},
   /* E7 */   { "ISC",  1, ZP    , 5, 0, READOP,   0},
   /* E8 */   { "INX",  0, IMP   , 2, 0, READOP,   op_INX},
   /* E9 */   { "SBC",  0, IMM   , 2, 0, READOP,   op_SBC},
   /* EA */   { "NOP",  0, IMP   , 2, 0, READOP,   0},
   /* EB */   { "SBC",  1, IMM   , 2, 0, READOP,   0},
   /* EC */   { "CPX",  0, ABS   , 4, 0, READOP,   op_CPX},
   /* ED */   { "SBC",  0, ABS   , 4, 0, READOP,   op_SBC},
   /* EE */   { "INC",  0, ABS   , 6, 0, RMWOP,    op_INC},
   /* EF */   { "ISC",  1, ABS   , 6, 0, READOP,   0},
   /* F0 */   { "BEQ",  0, BRA   , 2, 0, BRANCHOP, op_BEQ},
   /* F1 */   { "SBC",  0, INDY  , 5, 0, READOP,   op_SBC},
   /* F2 */   { "KIL",  1, IMP   , 0, 0, READOP,   0},
   /* F3 */   { "ISC",  1, INDY  , 8, 0, READOP,   0},
   /* F4 */   { "NOP",  1, ZPX   , 4, 0, READOP,   0},
   /* F5 */   { "SBC",  0, ZPX   , 4, 0, READOP,   op_SBC},
   /* F6 */   { "INC",  0, ZPX   , 6, 0, RMWOP,    op_INC},
   /* F7 */   { "ISC",  1, ZPX   , 6, 0, READOP,   0},
   /* F8 */   { "SED",  0, IMP   , 2, 0, READOP,   op_SED},
   /* F9 */   { "SBC",  0, ABSY  , 4, 0, READOP,   op_SBC},
   /* FA */   { "NOP",  1, IMP   , 2, 0, READOP,   0},
   /* FB */   { "ISC",  1, ABSY  , 7, 0, READOP,   0},
   /* FC */   { "NOP",  1, ABSX  , 4, 0, READOP,   0},
   /* FD */   { "SBC",  0, ABSX  , 4, 0, READOP,   op_SBC},
   /* FE */   { "INC",  0, ABSX  , 7, 0, RMWOP,    op_INC},
   /* FF */   { "ISC",  1, ABSX  , 7, 0, READOP,   0}
};

static char ILLEGAL[] = "???";

void em_init(int support_c02, int support_rockwell, int support_undocumented) {
   int i;
   c02 = support_c02;
   instr_table = support_c02 ? instr_table_65c02 : instr_table_6502;
   // If not supporting the Rockwell C02 extensions, tweak the cycle countes
   if (support_c02 && !support_rockwell) {
      // x7 (RMB/SMB): 5 cycles -> 1 cycles
      // xF (BBR/BBS): 5 cycles -> 1 cycles
      for (i = 0x07; i <= 0xff; i+= 0x08) {
         instr_table[i].mnemonic = ILLEGAL;
         instr_table[i].mode     = IMP;
         instr_table[i].cycles   = 1;
         instr_table[i].optype   = READOP;
         instr_table[i].len      = 1;
      }
   }
   InstrType *instr = instr_table;
   for (i = 0; i < 256; i++) {
      // Remove the undocumented instructions, if not supported
      if (instr->undocumented && !support_undocumented) {
         instr->mnemonic = ILLEGAL;
         instr->mode     = IMP;
         instr->cycles   = 1;
      }
      // Copy the length and format from the address mode, for efficiency
      instr->len = addr_mode_table[instr->mode].len;
      instr->fmt = addr_mode_table[instr->mode].fmt;
      instr++;
   }
   for (i = 0; i < 0xffff; i++) {
      memory[i] = -1;
   }
}
