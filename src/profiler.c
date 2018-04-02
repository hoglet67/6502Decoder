#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "profiler.h"

#define DEBUG           0
#define DROP_UNDERFLOW  0

// Define profiling type
#define TYPE_INSTR      1   // profile each instruction
#define TYPE_CALL_INC   2   // profile function calls, including children
#define TYPE_CALL_EXC   3   // profile function calls, excluding children

// Slot for instructions that fall outside the region of interest
// (don't change this or lots of things will break!)
#define ROOT_CONTEXT    0x10000

// Constants for call based profiling
// (6502 stack can only hold 128 addresses)
#define CALL_STACK_SIZE 128

// Maximum length of bar of asterisks
#define BAR_WIDTH 50

static int profile_enabled;
static int profile_type = TYPE_INSTR;
static int profile_min;
static int profile_max;
static int profile_bucket;

static int call_stack[CALL_STACK_SIZE] = { ROOT_CONTEXT };
static int call_stack_index = 1;
static uint32_t profile_counts[ROOT_CONTEXT + 1];

void profiler_parse_opt(int key, char *arg, struct argp_state *state) {
   switch (key) {
   case 'p':
      profile_min    = 0x0000;
      profile_max    = 0xffff;
      profile_bucket = 1;
      if (arg && strlen(arg) > 0) {
         char *type   = strtok(arg, ",");
         char *min    = strtok(NULL, ",");
         char *max    = strtok(NULL, ",");
         char *bucket = strtok(NULL, ",");
         if (!strcasecmp(type, "instr")) {
            profile_type = TYPE_INSTR;
         } else if (!strcasecmp(type, "call") || !strcasecmp(type, "callinc") || !strcasecmp(type, "call_inc")) {
            profile_type = TYPE_CALL_INC;
         } else if (!strcasecmp(type, "callexc") || !strcasecmp(type, "call_exc")) {
            profile_type = TYPE_CALL_EXC;
         } else {
            argp_error(state, "unknown profiler type %s", type);
         }
         if (min && strlen(min) > 0) {
            profile_min = strtol(min, (char **)NULL, 16);
         }
         if (max && strlen(max) > 0) {
            profile_max = strtol(max, (char **)NULL, 16);
         }
         if (bucket && strlen(bucket) > 0) {
            profile_bucket = atoi(bucket);
         }
      }
      break;
   }
}

void profiler_init() {
   profile_enabled = 1;
   memset((void *)profile_counts, 0, sizeof(profile_counts));
}

void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   if (!profile_enabled) {
      return;
   }
   if (profile_type == TYPE_INSTR) {
      // =======================================
      // Instruction based profiling
      // =======================================
      int bucket = ROOT_CONTEXT;
      if (pc >= profile_min && pc <= profile_max) {
         if (profile_bucket < 2) {
            bucket = pc & 0xffff;
         } else {
            bucket = ((pc & 0xffff) / profile_bucket) * profile_bucket;
         }
      }
      profile_counts[bucket] += num_cycles;
   } else {
      // =======================================
      // Call based profiling
      // =======================================
      if (opcode == 0x20) {
         // TODO: What about interrupts
         if (call_stack_index < CALL_STACK_SIZE) {
            int addr = (op2 << 8 | op1) & 0xffff;
#if DEBUG
            printf("*** pushing %04x to %d\n", addr, call_stack_index);
#endif
            call_stack[call_stack_index++] = addr;
         } else {
            printf("warning: call stack overflowed, disabling further profiling\n");
            for (int i = 0; i < call_stack_index; i++) {
               printf("warning: stack[%3d] = %04x\n", i, call_stack[i]);
            }
            profile_enabled = 0;
         }
      }
      if (profile_type == TYPE_CALL_INC) {
         for (int i = 0; i < call_stack_index; i++) {
            profile_counts[call_stack[i]] += num_cycles;
         }
      } else {
         profile_counts[call_stack[call_stack_index - 1]] += num_cycles;
      }
      if (opcode == 0x60) {
         if (call_stack_index > 1) {
            call_stack_index--;
#if DEBUG
            printf("*** popping %d\n", call_stack_index);
#endif
         } else {
#if DROP_UNDERFLOW
            uint64_t dropped = 0;
            for (int i = 0; i <= ROOT_CONTEXT; i++) {
               dropped += profile_counts[i];
            }
            memset((void *)profile_counts, 0, sizeof(profile_counts));
            printf("warning: call stack underflowed, dropping %ld cycles\n", dropped);
#else
            printf("warning: call stack underflowed\n");
#endif
         }
      }
   }
}

void profiler_done() {
   uint32_t      *cycles;
   uint32_t   max_cycles = 0;
   uint64_t total_cycles = 0;
   double  total_percent = 0.0;
   double      bar_scale;

   if (profile_type == TYPE_CALL_INC) {
      max_cycles = profile_counts[ROOT_CONTEXT];
      total_cycles = profile_counts[ROOT_CONTEXT];
   } else {
      cycles = profile_counts;
      for (int addr = 0; addr <= ROOT_CONTEXT; addr++) {
         if (*cycles > max_cycles) {
            max_cycles = *cycles;
         }
         total_cycles += *cycles++;
      }
   }

   bar_scale = (double) BAR_WIDTH / (double) max_cycles;

   cycles = profile_counts;
   for (int addr = 0; addr <= ROOT_CONTEXT; addr++) {
      if (*cycles) {
         double percent = 100.0 * (*cycles) / (double) total_cycles;
         total_percent += percent;
         if (addr == ROOT_CONTEXT) {
            printf("****");
         } else {
            printf("%04x", addr);
         }
         printf(" : %8d (%10.6f%%) ", (*cycles), percent);
         for (int i = 0; i < (int) (bar_scale * (*cycles)); i++) {
            printf("*");
         }
         printf("\n");
      }
      cycles++;
   }
   if (profile_type != TYPE_CALL_INC) {
      printf("     : %8ld (%10.6f%%)\n", total_cycles, total_percent);
   }
}
