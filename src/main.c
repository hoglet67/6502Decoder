#include <stdio.h>
#include <inttypes.h>

#include "em_6502.h"

enum Cycle {
   Cycle_FETCH,
   Cycle_OP1,
   Cycle_OP2,
   Cycle_MEMRD,
   Cycle_MEMWR
};

// Sync-loess decoder queue depth (samples)
// (min of 3 needed to reliably detect interrupts)
#define DEPTH 3

#define BUFSIZE 8192

uint16_t buffer[BUFSIZE];

// Whether to emulate each decoded instruction, to track additional state (registers and flags)

const int do_emulate = 1;

// TODO: all the pc prediction stuff could be pushed down into the emulation

// Predicted PC value
int pc = -1;

static void analyze_cycle(int opcode, int op1, int op2, int read_accumulator, int write_accumulator, int write_count, int operand, int num_cycles) {

   int offset;
   char target[16];

   // lookup the entry for the instruction
   InstrType *instr = &instr_table[opcode];

   // For instructions that push the current address to the stack we
   // can use the stacked address to determine the current PC
   int newpc = -1;
   if (write_count == 3) {
      // IRQ/NMI/RST
      newpc = (write_accumulator >> 8) & 0xffff;
   } else if (opcode == 0x20) {
      // JSR
      newpc = (write_accumulator - 2) & 0xffff;
   }

   // Sanity check the current pc prediction has not gone awry
   if (newpc >= 0) {
      if (pc >= 0 && pc != newpc) {
         printf("pc: prediction failed at %04X old pc was %04X\n", newpc, pc);
         pc = newpc;
      }
   }

   if (pc < 0) {
      printf(" ????: ");
   } else {
      printf(" %04X: ", pc);
   }

   int numchars = 0;
   if (write_count == 3 && opcode != 0) {
      // Annotate an interrupt
      numchars = printf("INTERRUPT !!");
      if (do_emulate) {
         em_interrupt(write_accumulator & 0xff);
      }
   } else {
      // Annotate a normal instruction
      const char *mnemonic = instr->mnemonic;
      const char *fmt = instr->fmt;
      switch (instr->mode) {
      case IMP:
      case IMPA:
         numchars = printf(fmt, mnemonic);
         break;
      case BRA:
      case ZPR:
         // Calculate branch target using op1 for normal branches and op2 for BBR/BBS
         offset = (int8_t) (((opcode & 0x0f) == 0x0f)  ? op2 : op1);
         if (pc < 0) {
            if (offset < 0) {
               sprintf(target, "pc-%d", -offset);
            } else {
               sprintf(target,"pc-%d", offset);
            }
         } else {
            sprintf(target, "%04X", pc + 2 + offset);
         }
         if (instr->mode == BRA) {
            numchars = printf(fmt, mnemonic, target);
         } else {
            numchars = printf(fmt, mnemonic, op1, target);
         }
         break;
      case IMM:
      case ZP:
      case ZPX:
      case ZPY:
      case INDX:
      case INDY:
      case IND:
         numchars = printf(fmt, mnemonic, op1);
         break;
      case ABS:
      case ABSX:
      case ABSY:
      case IND16:
      case IND1X:
         numchars = printf(fmt, mnemonic, op1, op2);
         break;
      }

      // Emulate the instruction
      if (do_emulate) {
         if (instr->emulate) {
            if (opcode == 0x40) {
               // special case RTI, operand (flags) is the first read cycle of three
               operand = read_accumulator & 0xff;
            }
            if (instr->optype == WRITEOP) {
               // special case instructions where the operand is being written (STA/STX/STY/PHP/PHA/PHX/PHY/BRK)
               operand = write_accumulator & 0xff;
            } else if (instr->optype == BRANCHOP) {
               // special case branch instructions, operand is true if branch taken
               operand = (num_cycles != 2);
            }
            instr->emulate(operand);
         }
      }
   }

   if (do_emulate) {
      // Pad opcode to 14 characters, to match python
      while (numchars++ < 14) {
         printf(" ");
      }
      printf("%s\n", em_get_state());
   } else {
      printf("\n");
   }

   // Look for control flow changes and update the PC
   if (opcode == 0x40 || opcode == 0x00 || opcode == 0x6c || opcode == 0x7c || write_count == 3) {
      // RTI, BRK, INTR, JMP (ind), JMP (ind, X), IRQ/NMI/RST
      pc = (read_accumulator >> 8) & 0xffff;
   } else if (opcode == 0x20 || opcode == 0x4c) {
      // JSR abs, JMP abs
      pc = op2 << 8 | op1;
   } else if (opcode == 0x60) {
      // RTS
      pc = (read_accumulator + 1) & 0xffff;
   } else if (pc < 0) {
      // PC value is not known yet, everything below this point is relative
      pc = -1;
   } else if (opcode == 0x80) {
      // BRA
      pc += ((int8_t)(op1)) + 2;
   } else if ((opcode & 0x0f) == 0x0f && num_cycles != 2) {
      // BBR/BBS
      pc += ((int8_t)(op2)) + 2;
   } else if ((opcode & 0x1f) == 0x10 && num_cycles != 2) {
      // BXX: op1 if taken
      pc += ((int8_t)(op1)) + 2;
   } else {
      // Otherwise, increment pc by length of instuction
      pc += instr->len;
   }
}


void decode_cycle_without_sync(int *bus_data_q, int *pin_rnw_q) {

   // Count of the 6502 bus cycles
   static int cyclenum             = 0;

   // Cycle count of the last sync, so we know the instruction cycle count
   static int last_cyclenum        = 0;

   // State to decode the 6502 bus activity
   static int opcode               = -1;
   static int op1                  = 0;
   static int op2                  = 0;
   static int read_accumulator     = 0;
   static int write_accumulator    = 0;
   static int write_count          = 0;
   static int operand              = 0;
   static int bus_cycle            = 0;
   static int cycle_count          = 0;
   static int opcount              = 0;

   int bus_data = *bus_data_q;
   int pin_rnw = *pin_rnw_q;

   // TODO: Find a more correct way of starting up!
   InstrType *instr = &instr_table[opcode >= 0 ? opcode : 0xEA];

   if (bus_cycle > cycle_count) {
      printf("cycle count error, %d %d\n", bus_cycle, cycle_count);
   }

   if (bus_cycle == 1 && opcount >= 1) {
      op1 = bus_data;
   }

   if (bus_cycle == 2 && opcount >= 2) {
      op2 = bus_data;
   }

   // Account for extra cycle in a page crossing in (indirect), Y
   if (bus_cycle == 4) {
      // Applies to ABSX and ABSY, but need to exclude stores
      if ((instr->mode == INDY) && (instr->optype == READOP)) {
         int index = em_get_Y();
         if (index >= 0) {
            int base = (read_accumulator >> 8) & 0xffff;
            if ((base & 0xff00) != ((base + index) & 0xff00)) {
               cycle_count ++;
            }
         }
      }
   }

   // Account for extra cycle in a page crossing in absolute indexed
   if (bus_cycle == 2) {
      // Applies to ABSX and ABSY, but need to exclude stores
      if (((instr->mode == ABSX) || (instr->mode == ABSY)) && (instr->optype == READOP)) {
         // Also need to exclude DEC and INC, which are 7 cycles regardless
         if ((opcode != 0xDE) && (opcode != 0xFE)) {
            int index = (instr->mode == ABSX) ? em_get_X() : em_get_Y();
            if (index >= 0) {
               int base = op1 + (op2 << 8);
               if ((base & 0xff00) != ((base + index) & 0xff00)) {
                  cycle_count ++;
               }
            }
         }
      }
   }

   // Account for extra cycles in a branch
   if (bus_cycle == 1) {
      if (((opcode & 0x1f) == 0x10) || (opcode == 0x80))
      {
         // Default to backards branches taken, forward not taken
         int taken = ((int8_t)op1) < 0;
         switch (opcode) {
         case 0x10: // BPL
            if (em_get_N() >= 0) {
               taken = !em_get_N();
            }
            break;
         case 0x30: // BMI
            if (em_get_N() >= 0) {
               taken = em_get_N();
            }
            break;
         case 0x50: // BVC
            if (em_get_V() >= 0) {
               taken = !em_get_V();
            }
            break;
         case 0x70: // BVS
            if (em_get_V() >= 0) {
               taken = em_get_V();
            }
            break;
         case 0x80: // BRA
            taken = 1;
            break;
         case 0x90: // BCC
            if (em_get_C() >= 0) {
               taken = !em_get_C();
            }
            break;
         case 0xB0: // BCS
            if (em_get_C() >= 0) {
               taken = em_get_C();
            }
            break;
         case 0xD0: // BNE
            if (em_get_Z() >= 0) {
               taken = !em_get_Z();
            }
            break;
         case 0xF0: // BEQ
            if (em_get_Z() >= 0) {
               taken = em_get_Z();
            }
            break;
         }
         if (taken) {
            // A taken branch is 3 cycles, not 2
            cycle_count = 3;
            // A taken branch that crosses a page boundary is 4 cycle
            if (pc >= 0) {
               int target =  (pc + 2) + ((int8_t)(op1));
               if ((target & 0xFF00) != ((pc + 2) & 0xff00)) {
                  cycle_count = 4;
               }
            }
         }
      }
   }

   // An interrupt sequence looks like:
   // 0 <opcode>            Rd
   // 1 <opcode>            Rd // address not incremented
   // 2 <return address hi> Wr
   // 3 <return address lo> Wr
   // 4 <flags>             Wr
   // 5 <vector lo>         Rd
   // 6 <vector hu>         Rd
   // 7 <opcode>            Rd

   // Detect interrupts as early as possible...
   if ((pin_rnw == 0) && (*(pin_rnw_q + 1) == 0) && (*(pin_rnw_q + 2) == 0)) {
         cycle_count = 7;
   }

   if (bus_cycle == cycle_count) {

      // Analyze the  previous instrucution
      if (opcode >= 0) {
         analyze_cycle(opcode, op1, op2, read_accumulator, write_accumulator, write_count, operand, cyclenum - last_cyclenum);
      }
      last_cyclenum  = cyclenum;

      // Re-initialize the state for the new instruction
      opcode            = bus_data;
      op1               = 0;
      op2               = 0;
      read_accumulator  = 0;
      write_accumulator = 0;
      write_count       = 0;
      operand           = 0;
      bus_cycle         = 0;
      cycle_count       = instr_table[opcode].cycles ;
      opcount           = instr_table[opcode].len - 1;

   } else {

      if (pin_rnw == 0) {
         write_count ++;
         write_accumulator = (write_accumulator << 8) | bus_data;
      } else {
         operand = bus_data;
         read_accumulator = (read_accumulator >> 8) | (bus_data << 16);
      }
   }

   // JSR is <opcode> <op1> <dummp stack rd> <stack wr> <stack wr> <op2>
   if (opcode == 0x20) {
      op2 = bus_data;
   }

   bus_cycle++;

   // Increment the cycle number (used only to detect taken branches)
   cyclenum ++;
}

#ifdef DEBUG
int sample_count = 0;
#endif

void lookahead_decode_cycle_without_sync(int bus_data, int pin_rnw) {
   static int bus_data_q[DEPTH];
   static int pin_rnw_q[DEPTH];
   static int fill = 0;

   bus_data_q[fill] = bus_data;
   pin_rnw_q[fill] = pin_rnw;
   if (fill < DEPTH - 1) {
      fill++;
   } else {
      decode_cycle_without_sync(bus_data_q, pin_rnw_q);
#ifdef DEBUG
      printf("%d %02x %d\n", sample_count, *bus_data_q, *pin_rnw_q);
#endif
      for (int i = 0; i < DEPTH - 1; i++) {
         bus_data_q[i] = bus_data_q[i + 1];
         pin_rnw_q[i] = pin_rnw_q[i + 1];
      }
   }
}

void decode_cycle_with_sync(int bus_data, int pin_rnw, int pin_sync) {

   // Count of the 6502 bus cycles
   static int cyclenum             = 0;

   // Cycle count of the last sync, so we know the instruction cycle count
   static int last_cyclenum        = 0;

   // State to decode the 6502 bus activity
   static int cycle                = Cycle_MEMRD;
   static int opcode               = -1;
   static int opcount              = 0;
   static int op1                  = 0;
   static int op2                  = 0;
   static int operand              = 0;
   static int write_count          = 0;
   static int read_accumulator     = 0;
   static int write_accumulator    = 0;

   if (pin_sync == 1) {

      // Sync indicates the start of a new instruction, the following variables pertain to the previous instruction
      // opcode, op1, op2, read_accumulator, write_accumulator, write_count, operand

      // Analyze the  previous instrucution
      if (opcode >= 0) {
         analyze_cycle(opcode, op1, op2, read_accumulator, write_accumulator, write_count, operand, cyclenum - last_cyclenum);
      }
      last_cyclenum  = cyclenum;

      // Re-initialize the state for the new instruction
      cycle             = Cycle_FETCH;
      opcode            = bus_data;
      opcount           = instr_table[opcode].len - 1;
      write_count       = 0;
      operand           = -1;
      read_accumulator  = 0;
      write_accumulator = 0;

   } else if (pin_rnw == 0) {
      cycle = Cycle_MEMWR;
      write_count ++;
      write_accumulator = (write_accumulator << 8) | bus_data;

   } else if (cycle == Cycle_FETCH && opcount > 0) {
      cycle = Cycle_OP1;
      opcount -= 1;
      op1 = bus_data;
      operand = bus_data;

   } else if (cycle == Cycle_OP1 && opcount > 0) {
      if (opcode == 0x20) { // JSR is <opcode> <op1> <dummp stack rd> <stack wr> <stack wr> <op2>
         cycle = Cycle_MEMRD;
      } else {
         cycle = Cycle_OP2;
         opcount -= 1;
         op2 = bus_data;
      }

   } else {
      if (opcode == 0x20) { // JSR, see above
         cycle = Cycle_OP2;
         opcount -= 1;
         op2 = bus_data;
      } else {
         cycle = Cycle_MEMRD;
         operand = bus_data;
         read_accumulator = (read_accumulator >> 8) | (bus_data << 16);
      }
   }

   // Increment the cycle number (used only to detect taken branches)
   cyclenum ++;
}

void decode(FILE *stream) {

   // Pin mappings into the 16 bit words
   // TODO: make these configurable
   int idx_data  =  0;
   int idx_rnw   =  8;
   int idx_sync  = -1;
   int idx_rdy   = 10;
   int idx_phi2  = 11;

   // Pin values
   int bus_data  =  0;
   int pin_rnw   =  0;
   int pin_sync  =  0;
   int pin_rdy   =  1;
   int pin_phi2  =  0;

   int num;

   // The previous sample of the 16-bit capture (async sampling only)
   uint16_t last_sample = -1;

   // The previous sample of phi2 (async sampling only)
   int last_phi2 = -1;
   int last2_phi2 = -1;

   while ((num = fread(buffer, sizeof(uint16_t), BUFSIZE, stream)) > 0) {

      uint16_t *sampleptr = &buffer[0];

      while (num-- > 0) {

         // The current 16-bit capture sample
         uint16_t sample = *sampleptr++;
#ifdef DEBUG
         printf("%02x %x %x %x %x\n", sample&255, (sample >> 8)&1,  (sample >> 9)&1,  (sample >> 10)&1,  (sample >> 11)&1  );
         sample_count++;
#endif

         // Phi2 is optional
         // - if asynchronous capture is used, it must be connected
         // - if synchronous capture is used, it must not connected
         if (idx_phi2 < 0) {

            // If Phi2 is not present, use the pins directly
            bus_data = (sample >> idx_data) & 255;
            pin_rnw = (sample >> idx_rnw ) & 1;
            if (idx_sync >= 0) {
               pin_sync = (sample >> idx_sync) & 1;
            }
            if (idx_rdy >= 0) {
               pin_rdy = (sample >> idx_rdy ) & 1;
            }

         } else {

            // If Phi2 is present, look for the falling edge, and proceed with the previous sample
            pin_phi2 = (sample >> idx_phi2) & 1;
            if (pin_phi2 == 1 || last_phi2 != 1 || last2_phi2 != 1) {
               last2_phi2 = last_phi2;
               last_phi2 = pin_phi2;
               last_sample = sample;
               continue;
            }

            // At this point, last2_phi2, last_phi2 = 1 and phi2 = 0 (i.e. falling edge)
            last_phi2 = 0;
            last2_phi2 = 0;

            // Sample the control signals before the clock edge
            pin_rnw = (last_sample >> idx_rnw ) & 1;
            if (idx_sync >= 0) {
               pin_sync = (last_sample >> idx_sync) & 1;
            }
            if (idx_rdy >= 0) {
               pin_rdy = (last_sample >> idx_rdy ) & 1;
            }
            // Sample write data before the clock edge, and read data after the clock edge
            bus_data  = ((pin_rnw ? sample : last_sample) >> idx_data ) & 255;
         }

         // Ignore the cycle if RDY is low
         if (pin_rdy == 0)
            continue;

         if (idx_sync < 0) {
            lookahead_decode_cycle_without_sync(bus_data, pin_rnw);
         } else {
            decode_cycle_with_sync(bus_data, pin_rnw, pin_sync);
         }
      }
   }
}

int main(int argc, char *argv[]) {
   em_init();
   if (argc != 2) {
      fprintf(stderr, "usage: %s <capture file>\n", argv[0]);
      return 1;
   }
   FILE *stream = fopen(argv[1], "r");
   if (stream == NULL) {
      perror("failed to open capture file");
      return 2;
   }
   decode(stream);
   fclose(stream);
   return 0;
}
