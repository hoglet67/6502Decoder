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

typedef struct {
   profiler_t profiler;
   int profile_min;
   int profile_max;
   int profile_bucket;
   uint32_t profile_counts[OTHER_CONTEXT + 1];
} profiler_instr_t;

static void p_init(void *ptr) {
   profiler_instr_t *instance = (profiler_instr_t *)ptr;
   memset((void *)instance->profile_counts, 0, sizeof(instance->profile_counts));
}

static void p_profile_instruction(void *ptr, int pc, int opcode, int op1, int op2, int num_cycles) {
   profiler_instr_t *instance = (profiler_instr_t *)ptr;
   int bucket = OTHER_CONTEXT;
   if (pc >= instance->profile_min && pc <= instance->profile_max) {
      if (instance->profile_bucket < 2) {
         bucket = pc & 0xffff;
      } else {
         bucket = ((pc & 0xffff) / instance->profile_bucket) * instance->profile_bucket;
      }
   }
   instance->profile_counts[bucket] += num_cycles;
}

static void p_done(void *ptr) {
   profiler_instr_t *instance = (profiler_instr_t *)ptr;
   uint32_t      *cycles;
   uint32_t   max_cycles = 0;
   uint64_t total_cycles = 0;
   double  total_percent = 0.0;
   double      bar_scale;

   cycles = instance->profile_counts;
   for (int addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (*cycles > max_cycles) {
         max_cycles = *cycles;
      }
      total_cycles += *cycles++;
   }

   bar_scale = (double) BAR_WIDTH / (double) max_cycles;

   cycles = instance->profile_counts;
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

void *profiler_instr_create(char *arg) {

   profiler_instr_t *instance = (profiler_instr_t *)calloc(1, sizeof(profiler_instr_t));

   instance->profiler.name                = "instr";
   instance->profiler.arg                 = strdup(arg);
   instance->profiler.init                = p_init;
   instance->profiler.profile_instruction = p_profile_instruction;
   instance->profiler.done                = p_done;
   instance->profile_min                  = 0x0000;
   instance->profile_max                  = 0xffff;
   instance->profile_bucket               = 1;

   if (arg && strlen(arg) > 0) {
      char *min    = strtok(arg, ",");
      char *max    = strtok(NULL, ",");
      char *bucket = strtok(NULL, ",");
      if (min && strlen(min) > 0) {
         instance->profile_min = strtol(min, (char **)NULL, 16);
      }
      if (max && strlen(max) > 0) {
         instance->profile_max = strtol(max, (char **)NULL, 16);
      }
      if (bucket && strlen(bucket) > 0) {
         instance->profile_bucket = strtol(bucket, (char **)NULL, 16);
      }
   }

   return instance;
}
