#ifndef DEFS
#define DEFS

#include <inttypes.h>

// Sample_type_t is an abstraction of both the 6502 SYNC and the 65816 VDA/VPA

typedef enum {     // 6502 Sync    65815 VDA/VPA
   UNKNOWN,        //      ?             ?   ?
   INTERNAL,       //      -             0   0
   PROGRAM,        //      -             0   1
   DATA,           //      0             1   0
   OPCODE,         //      1             1   1
   LAST            // a marker for the end of stream
} sample_type_t;

typedef struct {
   sample_type_t type;
   uint8_t       data;
   int8_t         rnw; // -1 indicates unknown
   int8_t         rst; // -1 indicates unknown
} sample_t;

#endif
