#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <search.h>

#include "profiler.h"

#define DEBUG           0

// Constants for call based profiling
// (6502 stack can only hold 128 addresses)
#define CALL_STACK_SIZE 128

typedef struct call_stack {
   int stack[CALL_STACK_SIZE];
   struct call_stack *parent;
   int index;
   uint64_t call_count;
   uint64_t cycle_count;
} call_stack_t;

static void *root = NULL;

static call_stack_t *current;

static uint64_t total_cycles;

static double total_percent;

static int profile_enabled;

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

static void p_parse_opt(char *arg) {
}

static void p_init() {
   call_stack_t *root_context = (call_stack_t *)malloc(sizeof(call_stack_t));
   root_context->index = 0;
   root_context->cycle_count = 0;
   root_context->call_count = 0;
   root_context->parent = NULL;
   if (root) {
      tdestroy(root, free);
   }
   root = NULL;
   current = *(call_stack_t **)tsearch(root_context, &root, compare_nodes);
   profile_enabled = 1;
}

void p_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   if (!profile_enabled) {
      return;
   }
   if (opcode == 0x20) {
      // TODO: What about interrupts
      if (current->index < CALL_STACK_SIZE) {
         int addr = (op2 << 8 | op1) & 0xffff;
#if DEBUG
         printf("*** pushing %04x to %d\n", addr, current->index);
#endif
         // Create a new child node, in case it's not already in the tree
         call_stack_t *child = (call_stack_t *) malloc(sizeof(call_stack_t));
         memcpy((void*) child, (void *)current, sizeof(call_stack_t));
         child->stack[child->index] = addr;
         child->index++;
         child->parent = current;
         child->cycle_count = 0;
         child->call_count = 0;
         current = *(call_stack_t **)tsearch(child, &root, compare_nodes);
         // If the child already existed, then free the just created node
         if (current != child) {
            free(child);
         }
         current->call_count++;
      } else {
         printf("warning: call stack overflowed, disabling further profiling\n");
         for (int i = 0; i < current->index; i++) {
            printf("warning: stack[%3d] = %04x\n", i, current->stack[i]);
         }
         profile_enabled = 0;
      }
   }
   current->cycle_count += num_cycles;
   if (opcode == 0x60) {
      if (current->parent) {
#if DEBUG
         printf("*** popping %d\n", current->index);
#endif
         current = current->parent;
      } else {
         printf("warning: call stack underflowed, re-initialize call graph\n");
         profiler_init();
      }
   }
}

static void print_node(const call_stack_t *node) {
   int first = 1;
   double percent = 100.0 * (double) node->cycle_count / (double) total_cycles;
   total_percent += percent;
   printf("%8ld cycles (%10.6f%%) %8ld calls: ", node->cycle_count, percent, node->call_count);
   for (int i = 0; i < node->index; i++) {
      if (!first) {
         printf("->");
      }
      first = 0;
      printf("%04X", node->stack[i]);
   }
   printf("\n");
}

static void count_call_walker(const void *nodep, const VISIT which, const int depth) {
   if (which == postorder || which == leaf) {
      total_cycles += (*(call_stack_t **)nodep)->cycle_count;
   }
}

static void dump_call_walker(const void *nodep, const VISIT which, const int depth) {
   if (which == postorder || which == leaf) {
      print_node(*(call_stack_t **)nodep);
   }
}

static void p_done() {
   total_cycles = 0;
   twalk(root, count_call_walker);
   total_percent = 0;
   twalk(root, dump_call_walker);
   printf("%8ld cycles (%10.6f%%)\n", total_cycles, total_percent);
}

profiler_t profiler_call = {
   .profiler_name       = "call",
   .parse_opt           = p_parse_opt,
   .init                = p_init,
   .profile_instruction = p_profile_instruction,
   .done                = p_done
};
