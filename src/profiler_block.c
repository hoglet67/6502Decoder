#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "profiler.h"

typedef struct {
   profiler_t profiler;
   int profile_min;
   int profile_max;
   address_t profile_counts[OTHER_CONTEXT + 1];
   int last_opcode;
   cpu_emulator_t *em;
} profiler_block_t;

static void p_init(void *ptr, cpu_emulator_t *em) {
   profiler_block_t *instance = (profiler_block_t *)ptr;
   memset((void *)instance->profile_counts, 0, sizeof(instance->profile_counts));
   instance->profile_counts[OTHER_CONTEXT].flags = 1;
   instance->em = em;
}

static void p_profile_instruction(void *ptr, int pc, int opcode, int op1, int op2, int num_cycles) {
   profiler_block_t *instance = (profiler_block_t *)ptr;
   int addr;
   if (pc >= instance->profile_min && pc <= instance->profile_max) {
      addr = pc & 0xffff;
   } else {
      addr = OTHER_CONTEXT;
   }
   instance->profile_counts[addr].instructions++;
   instance->profile_counts[addr].cycles += num_cycles;
   // Test the test instruction to catch the destination of an indiect JMP
   // (this will break if the following instruction is interrupted)
   if (instance->last_opcode == 0x6c) {
      instance->profile_counts[addr].flags |= FLAG_JMP_IND;
   } else if (instance->last_opcode == 0x7c) {
      instance->profile_counts[addr].flags |= FLAG_JMP_INDX;
   }
   if (opcode == 0x20) {
      // Note the destination of JSR <abs>
      addr = (op2 << 8 | op1) & 0xffff;
      instance->profile_counts[addr].flags |= FLAG_JSR;
   } else if (opcode == 0x4c) {
      // Note the destination of JMP <abs>
      addr = (op2 << 8 | op1) & 0xffff;
      instance->profile_counts[addr].flags |= FLAG_JMP;
   } else if (pc >= 0 && (((opcode & 0x1f) == 0x10) || (opcode == 0x80))) {
      // Note the destination of Bxx <rel>
      addr = ((pc + 2) + ((int8_t)(op1))) & 0xffff;
      instance->profile_counts[addr  ].flags |= addr < pc ? FLAG_BB_TAKEN : FLAG_FB_TAKEN;
      instance->profile_counts[pc + 2].flags |= addr < pc ? FLAG_BB_NOT_TAKEN : FLAG_FB_NOT_TAKEN;
   }
   instance->last_opcode = opcode;
}

static void p_done(void *ptr) {
   profiler_block_t *instance = (profiler_block_t *)ptr;
   address_t block_counts[OTHER_CONTEXT + 1];
   memset((void *)block_counts, 0, sizeof(block_counts));
   int current_block = OTHER_CONTEXT;
   int addr;
   for (addr = 0; addr <= OTHER_CONTEXT; addr++) {
      if (instance->profile_counts[addr].flags) {
         current_block = addr;
         block_counts[current_block].flags = instance->profile_counts[addr].flags;
         block_counts[current_block].calls = instance->profile_counts[addr].instructions;
      }
      block_counts[current_block].cycles += instance->profile_counts[addr].cycles;
      block_counts[current_block].instructions += instance->profile_counts[addr].instructions;
   }
   profiler_output_helper(block_counts, 0, 1, NULL);
}

void *profiler_block_create(char *arg) {

   profiler_block_t *instance = (profiler_block_t *)calloc(1, sizeof(profiler_block_t));

   instance->profiler.name                = "block";
   instance->profiler.arg                 = arg ? strdup(arg) : "";
   instance->profiler.init                = p_init;
   instance->profiler.profile_instruction = p_profile_instruction;
   instance->profiler.done                = p_done;
   instance->profile_min                  = 0x0000;
   instance->profile_max                  = 0xffff;

   if (arg && strlen(arg) > 0) {
      char *min    = strtok(arg, ",");
      char *max    = strtok(NULL, ",");
      if (min && strlen(min) > 0) {
         instance->profile_min = strtol(min, (char **)NULL, 16);
      }
      if (max && strlen(max) > 0) {
         instance->profile_max = strtol(max, (char **)NULL, 16);
      }
   }

   return instance;
}
