#include <string.h>

#include "profiler.h"

extern profiler_t *profiler_instr_create(char *arg);
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

void profiler_init() {
   profiler_t **pp = active_list;
   while (*pp) {
      (*pp)->init(*pp);
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
