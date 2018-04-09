#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <search.h>

#include "profiler.h"

#define DEBUG           0

// Define profiling type
#define TYPE_INSTR      1   // profile each instruction
#define TYPE_CALL       2   // profile function calls

// Slot for instructions that fall outside the region of interest
// (don't change this or lots of things will break!)
#define OTHER_CONTEXT    0x10000

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

static uint32_t profile_counts[OTHER_CONTEXT + 1];

typedef struct call_stack {
   int stack[CALL_STACK_SIZE];
   struct call_stack *parent;
   int index;
   uint64_t call_count;
   uint64_t cycle_count;
} call_stack_t;

static void *root = NULL;

static call_stack_t *current;

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

static void profiler_init_call_graph() {
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
}

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
         } else if (!strcasecmp(type, "call")) {
            profile_type = TYPE_CALL;
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
            profile_bucket = strtol(bucket, (char **)NULL, 16);
         }
      }
      break;
   }
}

void profiler_init() {
   profile_enabled = 1;
   memset((void *)profile_counts, 0, sizeof(profile_counts));
   profiler_init_call_graph();
}

void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   if (!profile_enabled) {
      return;
   }
   if (profile_type == TYPE_INSTR) {
      // =======================================
      // Instruction based profiling
      // =======================================
      int bucket = OTHER_CONTEXT;
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
            profiler_init_call_graph();
         }
      }
   }
}

// ==========================================================
// Outputter for hierarchical call profiling
// ==========================================================

static uint64_t total_cycles;
static double total_percent;

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

static void profiler_count_call_walker(const void *nodep, const VISIT which, const int depth) {
   if (which == postorder || which == leaf) {
      total_cycles += (*(call_stack_t **)nodep)->cycle_count;
   }
}

static void profiler_dump_call_walker(const void *nodep, const VISIT which, const int depth) {
   if (which == postorder || which == leaf) {
      print_node(*(call_stack_t **)nodep);
   }
}

static void profiler_dump_call() {
   total_cycles = 0;
   twalk(root, profiler_count_call_walker);
   total_percent = 0;
   twalk(root, profiler_dump_call_walker);
   printf("%8ld (%10.6f%%)\n", total_cycles, total_percent);
}

// ==========================================================
// Outputter for flat instruction profiling
// ==========================================================

static void profiler_dump_instr() {
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

void profiler_done() {
   if (profile_type == TYPE_INSTR) {
      profiler_dump_instr();
   } else {
      profiler_dump_call();
   }
}
