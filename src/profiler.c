#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#define CALL_STACK_SIZE 128

#define ROOT_CONTEXT   0x10000

#define BUCKET_SIZE 1

static int call_profiling = 1;
static int call_stack[CALL_STACK_SIZE] = { ROOT_CONTEXT };
static int call_stack_index = 1;
static uint32_t profile_counts[0x10000 + 1];
static int profile;
static int profile_min;
static int profile_max;

void profiler_parse_args(char *arg) {
   profile_min = 0x0000;
   profile_max = 0xffff;
   if (arg && strlen(arg) > 0) {
      char *min = strtok(arg, ",");
      char *max = strtok(NULL, ",");
      if (min && strlen(min) > 0) {
         profile_min = strtol(min, (char **)NULL, 16);
      }
      if (max && strlen(max) > 0) {
         profile_max = strtol(max, (char **)NULL, 16);
      }
   }
}

void profiler_init() {
   profile = 1;
   memset((void *)profile_counts, 0, sizeof(profile_counts));
}

void profiler_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   if (!profile) {
      return;
   }
   if (call_profiling) {
      if (opcode == 0x20) {
         // TODO: What about interrupts
         if (call_stack_index < CALL_STACK_SIZE) {
            int addr = (op2 << 8 | op1) & 0xffff;
            //printf("*** pushing %04x to %d\n", addr, call_stack_index);
               call_stack[call_stack_index++] = addr;
         } else {
            printf("fail: call stack overflowed, disabling further profiling\n");
            //for (i = 0; i < call_stack_index; i++) {
            //   printf("stack[%3d] = %04x\n", i, call_stack[i]);
            //}
            profile = 0;
         }
      }
      //for (i = 0; i < call_stack_index; i++) {
      //   profile_counts[call_stack[i]] += num_cycles;
      //}
      profile_counts[call_stack[call_stack_index - 1]] += num_cycles;
      if (opcode == 0x60) {
         if (call_stack_index > 1) {
            call_stack_index--;
            //printf("*** popping %d\n", call_stack_index);
         } else {
            //uint64_t dropped = 0;
            //for (int i = 0; i <= 0x10000; i++) {
            //   dropped += profile_counts[i];
            //}
            //memset((void *)profile_counts, 0, sizeof(profile_counts));
            //printf("fail: call stack underflowed, dropping %ld cycles\n", dropped);
            printf("fail: call stack underflowed\n");
         }
      }
   } else {
      if (pc >= profile_min && pc <= profile_max) {
         profile_counts[pc & 0xffff] += num_cycles;
      } else {
         profile_counts[0x10000] += num_cycles;
      }
   }
}

void profiler_done() {
   int addr;
   uint64_t total_cycles = 0;
   double total_percent = 0.0;
   uint32_t *cycles;
   int i;
   uint32_t bucket;

   cycles = profile_counts;
   for (addr = 0; addr <= 0x10000; addr++) {
      total_cycles += *cycles++;
   }

   cycles = profile_counts;
   for (addr = 0; addr <= 0x10000; addr += BUCKET_SIZE) {
      bucket = 0;
      for (i = 0; i < BUCKET_SIZE; i++) {
         bucket += *cycles++;
      }
      if (bucket) {
         double percent = 100.0 * bucket / (double) total_cycles;
         total_percent += percent;
         if (addr < 0x10000) {
            printf("%04x", addr);
         } else {
            printf("????");
         }
         printf(" : %8d (%10.6f%%) ", bucket, percent);
         for (i = 0; i < percent; i++) {
            printf("*");
         }
         printf("\n");
      }
   }
   printf("     : %8ld (%10.6f%%)\n", total_cycles, total_percent);
}
