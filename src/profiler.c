#include <string.h>

#include "profiler.h"

extern profiler_t profiler_instr;

extern profiler_t profiler_call;

static profiler_t *profiler_list[] = {
   &profiler_instr,
   &profiler_call,
   NULL
};

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
         profiler_t **pp = profiler_list;
         while (*pp) {
            if (strcasecmp(type, (*pp)->profiler_name) == 0) {
               break;
            }
            pp++;
         }
         if (*pp) {
            (*pp)->parse_opt(rest);
            active_list[active_count++] = *pp;
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
      (*pp++)->init();
   }
}

void profiler_profile_instruction(int pc, int opcode, int op1, int op2, int num_cycles) {
   profiler_t **pp = active_list;
   while (*pp) {
      (*pp++)->profile_instruction(pc, opcode, op1, op2, num_cycles);
   }
}

void profiler_done() {
   profiler_t **pp = active_list;
   while (*pp) {
      (*pp++)->done();
   }
}
