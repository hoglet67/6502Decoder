#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "musl_tsearch.h"
#include "profiler.h"
#include "symbols.h"

#define DEBUG           0

// Constants for call based profiling
// (6502 stack can only hold 128 addresses)
#define CALL_STACK_SIZE 128

static cpu_emulator_t *my_em;

typedef struct call_stack {
   int stack[CALL_STACK_SIZE];
   struct call_stack *parent;
   int index;
   uint64_t call_count;
   uint64_t cycle_count;
} call_stack_t;


typedef struct {
   profiler_t profiler;
   void *root;
   call_stack_t *current;
   int profile_enabled;
   cpu_emulator_t *em;
} profiler_call_t;


// One copy of these means profile_done is not thread safe, not concerned about this
static uint64_t total_cycles;
static double total_percent;

static int compare_nodes(const void *av, const void *bv) {
   const call_stack_t *a = (call_stack_t *) av;
   const call_stack_t *b = (call_stack_t *) bv;
   int ret = 0;
   for (int i = 0; i < a->index && i < b->index; i++) {
      if (a->stack[i] < b->stack[i]) {
         ret = -1;
         break;
      } else if (a->stack[i] > b->stack[i]) {
         ret = 1;
         break;
      }
   }
   if (ret == 0) {
      if (a->index < b->index) {
         ret = -1;
      } else if (a->index > b->index) {
         ret = 1;
      }
   }
   return ret;
}

static void p_init(void *ptr, cpu_emulator_t *em) {
   profiler_call_t *instance = (profiler_call_t *)ptr;
   call_stack_t *root_context = (call_stack_t *)malloc(sizeof(call_stack_t));
   root_context->index = 0;
   root_context->cycle_count = 0;
   root_context->call_count = 0;
   root_context->parent = NULL;
   if (instance->root) {
      ttdestroy(instance->root, free);
   }
   instance->root = NULL;
   instance->current = *(call_stack_t **)ttsearch(root_context, &instance->root, compare_nodes);
   instance->profile_enabled = 1;
   instance->em = em;
   my_em = em;
}

static void p_profile_instruction(void *ptr, int pc, int opcode, int op1, int op2, int num_cycles) {
   profiler_call_t *instance = (profiler_call_t *)ptr;
   if (!instance->profile_enabled) {
      return;
   }
   instance->current->cycle_count += num_cycles;
   if (opcode == 0x20) {
      // TODO: What about interrupts
      if (instance->current->index < CALL_STACK_SIZE) {
         int addr = (op2 << 8 | op1) & 0xffff;
#if DEBUG
         printf("*** pushing %04x to %d\n", addr, current->index);
#endif
         // Create a new child node, in case it's not already in the tree
         call_stack_t *child = (call_stack_t *) malloc(sizeof(call_stack_t));
         memcpy((void*) child, (void *)(instance->current), sizeof(call_stack_t));
         child->stack[child->index] = addr;
         child->index++;
         child->parent = instance->current;
         child->cycle_count = 0;
         child->call_count = 0;
         instance->current = *(call_stack_t **)ttsearch(child, &instance->root, compare_nodes);
         // If the child already existed, then free the just created node
         if (instance->current != child) {
            free(child);
         }
         instance->current->call_count++;
      } else {
         printf("warning: call stack overflowed, disabling further profiling\n");
         for (int i = 0; i < instance->current->index; i++) {
            printf("warning: stack[%3d] = %04x\n", i, instance->current->stack[i]);
         }
         instance->profile_enabled = 0;
      }
   }
   if (opcode == 0x60) {
      if (instance->current->parent) {
#if DEBUG
         printf("*** popping %d\n", current->index);
#endif
         instance->current = instance->current->parent;
      } else {
         printf("warning: call stack underflowed, re-initialize call graph\n");
         p_init(ptr, instance->em);
      }
   }
}

static void print_node(const call_stack_t *node) {
   int first = 1;
   double percent = 100.0 * (double) node->cycle_count / (double) total_cycles;
   total_percent += percent;
   printf("%8" PRIu64 " cycles (%10.6f%%) %8" PRIu64 " calls: ", node->cycle_count, percent, node->call_count);
   for (int i = 0; i < node->index; i++) {
      if (!first) {
         printf("->");
      }
      first = 0;
      char *name=symbol_lookup(node->stack[i]);
      if (name) {
         if (name[0] == '.') name++;
         printf("%s", name);
      } else {
         printf("%04X", node->stack[i]);
      }
   }
   printf("\n");
}

static void count_call_walker(const void *nodep, const TVISIT which, const int depth) {
   if (which == tpostorder || which == tleaf) {
      total_cycles += (*(call_stack_t **)nodep)->cycle_count;
   }
}

static void dump_call_walker(const void *nodep, const TVISIT which, const int depth) {
   if (which == tpostorder || which == tleaf) {
      print_node(*(call_stack_t **)nodep);
   }
}

static void p_done(void *ptr) {
   profiler_call_t *instance = (profiler_call_t *)ptr;
   total_cycles = 0;
   ttwalk(instance->root, count_call_walker);
   total_percent = 0;
   ttwalk(instance->root, dump_call_walker);
   printf("%8" PRIu64 " cycles (%10.6f%%)\n", total_cycles, total_percent);
}

void *profiler_call_create(char *arg) {
   profiler_call_t *instance = (profiler_call_t *)calloc(1, sizeof(profiler_call_t));

   instance->profiler.name                = "call";
   instance->profiler.arg                 = arg ? strdup(arg) : "";
   instance->profiler.init                = p_init;
   instance->profiler.profile_instruction = p_profile_instruction;
   instance->profiler.done                = p_done;

   return instance;
}
