#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "defs.h"
#include "symbols.h"

typedef enum {
   SWS_GROUND,
   SWS_GOT_SQUARE,
   SWS_GOT_CURLY,
   SWS_IN_NAME,
   SWS_TOO_LONG,
   SWS_NAME_END,
   SWS_IN_VALUE,
   SWS_AWAIT_COMMA
} swstate;

void swiftsym(cpu_emulator_t *cpu, char *iptr)
{
   if (!cpu->symbol_add) {
      fprintf(stderr, "emulator does not support symbol files\r\n");
      exit(1);
   }
   FILE *fp = fopen(iptr, "r");
   if (fp) {
      swstate state = SWS_GROUND;
      char name[80], *name_ptr, *name_end = name+sizeof(name);
      uint32_t addr;
      int ch, syms = 0;
      while ((ch = getc(fp)) != EOF) {
         switch(state) {
         case SWS_GROUND:
            if (ch == '[')
               state = SWS_GOT_SQUARE;
            break;
         case SWS_GOT_SQUARE:
            if (ch == '{')
               state = SWS_GOT_CURLY;
            else if (ch != '[')
               state = SWS_GROUND;
            break;
         case SWS_GOT_CURLY:
            if (ch == '\'') {
               name_ptr = name;
               state = SWS_IN_NAME;
            }
            else if (!strchr(" \t\r\n", ch))
               state = SWS_GROUND;
            break;
         case SWS_IN_NAME:
            if (ch == '\'') {
               *name_ptr = 0;
               state = SWS_NAME_END;
            }
            else if (name_ptr >= name_end) {
               fprintf(stderr, "swift import name too long");
               state = SWS_TOO_LONG;
            }
            else
               *name_ptr++ = ch;
            break;
         case SWS_TOO_LONG:
            if (ch == '\'') {
               name_ptr = 0;
               state = SWS_NAME_END;
            }
            break;
         case SWS_NAME_END:
            if (ch == ':') {
               addr = 0;
               state = SWS_IN_VALUE;
            }
            else if (!strchr(" \t\r\n", ch))
               state = SWS_GROUND;
            break;
         case SWS_IN_VALUE:
            if (ch >= '0' && ch <= '9')
               addr = addr * 10 + ch - '0';
            else if (ch == 'L') {
               cpu->symbol_add(name, addr);
               state = SWS_AWAIT_COMMA;
               syms++;
            }
            else if (ch == ',') {
               cpu->symbol_add(name, addr);
               state = SWS_GOT_CURLY;
               syms++;
            }
            else
               state = SWS_GROUND;
            break;
         case SWS_AWAIT_COMMA:
            if (ch == ',')
               state = SWS_GOT_CURLY;
            else if (!strchr(" \t\r\n", ch))
               state = SWS_GROUND;
         }
      }
      fclose(fp);
   }
   else
      fprintf(stderr, "unable to open '%s': %s\n", iptr, strerror(errno));
}
