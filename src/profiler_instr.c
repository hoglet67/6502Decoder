#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "profiler.h"

typedef struct {
   profiler_t profiler;
   int profile_min;
   int profile_max;
   int profile_bucket;
   address_t profile_counts[OTHER_CONTEXT + 1];
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
   instance->profile_counts[bucket].instructions++;
   instance->profile_counts[bucket].cycles += num_cycles;
}

static void p_done(void *ptr) {
   profiler_instr_t *instance = (profiler_instr_t *)ptr;
   profiler_output_helper(instance->profile_counts, 1, 0);
}

void *profiler_instr_create(char *arg) {

   profiler_instr_t *instance = (profiler_instr_t *)calloc(1, sizeof(profiler_instr_t));

   instance->profiler.name                = "instr";
   instance->profiler.arg                 = arg ? strdup(arg) : "";
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
