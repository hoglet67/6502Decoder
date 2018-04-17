#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "profiler.h"

// Slot for instructions that fall outside the region of interest
// (don't change this or lots of things will break!)
#define OTHER_CONTEXT    0x10000

// Maximum length of bar of asterisks
#define BAR_WIDTH 50

static int profile_min;
static int profile_max;
static int profile_bucket;

static uint32_t profile_counts[OTHER_CONTEXT + 1];

static void p_parse_opt(char *arg) {
   profile_min    = 0x0000;
   profile_max    = 0xffff;
   profile_bucket = 1;
   if (arg && strlen(arg) > 0) {
      char *min    = strtok(arg, ",");
      char *max    = strtok(NULL, ",");
      char *bucket = strtok(NULL, ",");
      if (min && strlen(min) > 0) {
         profile_min = strtol(min, (char **)NULL, 16);
      }
      if (max && strlen(max) > 0) {
         profile_max = strtol(max, (char **)NULL, 16);
      }
      if (bucket && strlen(bucket) > 0) {
         profile_bucket = strtol(bucket, (char **)NULL, 16);
      }
   }

}

static void p_init() {
   memset((void *)profile_counts, 0, sizeof(profile_counts));
}

static void p_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   int bucket = OTHER_CONTEXT;
   if (pc >= profile_min && pc <= profile_max) {
      if (profile_bucket < 2) {
         bucket = pc & 0xffff;
      } else {
         bucket = ((pc & 0xffff) / profile_bucket) * profile_bucket;
      }
   }
   profile_counts[bucket] += num_cycles;
}

static void p_done() {
   uint32_t      *cycles;
   uint32_t   max_cycles = 0;
   uint64_t total_cycles = 0;
   double  total_percent = 0.0;
   double      bar_scale;

   cycles = profile_counts;
   for (int addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (*cycles > max_cycles) {
         max_cycles = *cycles;
      }
      total_cycles += *cycles++;
   }

   bar_scale = (double) BAR_WIDTH / (double) max_cycles;

   cycles = profile_counts;
   for (int addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (*cycles) {
         double percent = 100.0 * (*cycles) / (double) total_cycles;
         total_percent += percent;
         if (addr == OTHER_CONTEXT) {
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
   printf("     : %8ld (%10.6f%%)\n", total_cycles, total_percent);
}

profiler_t profiler_instr = {
   .profiler_name       = "instr",
   .parse_opt           = p_parse_opt,
   .init                = p_init,
   .profile_instruction = p_profile_instruction,
   .done                = p_done
};
