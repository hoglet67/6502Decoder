#include <string.h>
#include <inttypes.h>

#include "profiler.h"

extern profiler_t *profiler_instr_create(char *arg);
extern profiler_t *profiler_block_create(char *arg);
extern profiler_t *profiler_call_create(char *arg);

#define MAX_PROFILERS 10

static profiler_t *active_list[MAX_PROFILERS] = { NULL } ;

void profiler_parse_opt(int key, char *arg, struct argp_state *state) {
static int active_count = 0;
   switch (key) {
   case 'p':
      if (arg && strlen(arg) > 0) {
         char *type   = strtok(arg, ",");
         char *rest   = strtok(NULL, "");
         // Act as a factory method for profilers
         profiler_t *instance = NULL;
         if (strcasecmp(type, "instr") == 0) {
            instance = profiler_instr_create(rest);
         } else if (strcasecmp(type, "block") == 0) {
            instance = profiler_block_create(rest);
         } else if (strcasecmp(type, "call") == 0) {
            instance = profiler_call_create(rest);
         }
         if (instance) {
            active_list[active_count++] = instance;
            active_list[active_count] = NULL;
         } else {
            argp_error(state, "unknown profiler type %s", type);
         }
      }
      break;
   }
}

void profiler_init(cpu_emulator_t *em) {
   profiler_t **pp = active_list;
   while (*pp) {
      (*pp)->init(*pp, em);
      pp++;
   }
}

void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   profiler_t **pp = active_list;
   while (*pp) {
      (*pp)->profile_instruction(*pp, pc, opcode, op1, op2, num_cycles);
      pp++;
   }
}

void profiler_done() {
   profiler_t **pp = active_list;
   while (*pp) {
   printf("==============================================================================\n");
   printf("Profiler: %s; Args: %s\n", (*pp)->name, (*pp)->arg);
   printf("==============================================================================\n");
      (*pp)->done(*pp);
      pp++;
   }
}

void profiler_output_helper(address_t *profile_counts, int show_bars, int show_other, cpu_emulator_t *em) {
   address_t      *ptr;

   uint32_t   max_cycles = 0;
   uint64_t total_cycles = 0;
   uint64_t  total_instr = 0;
   double  total_percent = 0.0;
   double      bar_scale;

   char buffer[256];

   ptr = profile_counts;

   for (int addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (ptr->cycles > max_cycles) {
         max_cycles = ptr->cycles;
      }
      total_cycles += ptr->cycles;
      total_instr += ptr->instructions;
      ptr++;
   }

   bar_scale = (double) BAR_WIDTH / (double) max_cycles;

   ptr = profile_counts;
   for (int addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (ptr->cycles) {
         double percent = 100.0 * (ptr->cycles) / (double) total_cycles;
         total_percent += percent;
         if (addr == OTHER_CONTEXT) {
            printf("****");
         } else {
            printf("%04x", addr);
            if (em) {
               instruction_t instruction;
               instruction.pc     = addr;
               instruction.opcode = em->read_memory(addr);
               instruction.op1    = em->read_memory(addr + 1);
               instruction.op2    = em->read_memory(addr + 2);
               int n = em->disassemble(buffer, &instruction);
               printf(" %s", buffer);
               for (int i = n; i < 12; i++) {
                  putchar(' ');
               }
            }
         }
         printf(" : %8d cycles (%10.6f%%) %8d ins (%4.2f cpi)", ptr->cycles, percent, ptr->instructions, (double) ptr->cycles / (double) ptr->instructions);
         if (show_other) {
            printf(" %8d calls", ptr->calls);
            printf(" (");
            printf(ptr->flags & FLAG_JSR           ? "J" : " ");
            printf(ptr->flags & FLAG_JMP           ? "j" : " ");
            printf(ptr->flags & FLAG_BB_TAKEN      ? "B" : " ");
            printf(ptr->flags & FLAG_FB_TAKEN      ? "F" : " ");
            printf(ptr->flags & FLAG_BB_NOT_TAKEN  ? "b" : " ");
            printf(ptr->flags & FLAG_FB_NOT_TAKEN  ? "f" : " ");
            printf(ptr->flags & FLAG_JMP_IND       ? "i" : " ");
            printf(ptr->flags & FLAG_JMP_INDX      ? "x" : " ");
            printf(")");
         }
         if (show_bars) {
            printf(" ");
            for (int i = 0; i < (int) (bar_scale * ptr->cycles); i++) {
               printf("*");
            }
         }
         printf("\n");
      }
      ptr++;
   }
   printf("     : %8" PRIu64 " cycles (%10.6f%%) %8" PRIu64 " ins (%4.2f cpi)\n", total_cycles, total_percent, total_instr, (double) total_cycles / (double) total_instr);
}
