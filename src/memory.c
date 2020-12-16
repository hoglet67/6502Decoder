#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "defs.h"
#include "tube_decode.h"
#include "memory.h"

// Sideways ROM

#define SWROM_START         0x8000
#define SWROM_ADDR_BITS     14
#define SWROM_ADDR_MASK     ((1 << SWROM_ADDR_BITS) - 1)
#define SWROM_SIZE          (1 << SWROM_ADDR_BITS)
#define SWROM_END           (SWROM_START + SWROM_SIZE)
#define SWROM_NUM_BANKS     16

static int *swrom         = NULL;
static int rom_latch_addr = -1;
static int rom_latch_data = -1;

// Main Memory

static int *memory        = NULL;
static int mem_model      = 0;
static int mem_rd_logging = 0;
static int mem_wr_logging = 0;
static int addr_digits    = 0;

// IO
static int io_low         = -1;
static int io_high        = -1;
static int tube_low       = -1;
static int tube_high      = -1;

static char buffer[256];

static inline int *get_memptr(int ea) {
   if (ea >= SWROM_START && ea < SWROM_END && rom_latch_data >= 0) {
      return swrom + (rom_latch_data << SWROM_ADDR_BITS) + (ea & SWROM_ADDR_MASK);
   } else {
      return memory + ea;
   }
}


static inline int write_addr(char *bp, int ea) {
   int shift = (addr_digits - 1) << 2; // 6 => 20
   for (int i = 0; i < addr_digits; i++) {
      int value = (ea >> shift) & 0xf;
      *bp++ = value + (value < 10 ? '0' : 'A' - 10);
      shift -= 4;
   }
   return addr_digits;
}


void memory_init(int size) {
   memory = malloc(size * sizeof(int));
   for (int i = 0; i < size; i++) {
      memory[i] = -1;
   }
   addr_digits = 0;
   // Calculate the number of digits to represent an address
   size--;
   while (size) {
      size >>= 1;
      addr_digits++;
   }
   addr_digits = (addr_digits + 3) >> 2;
}

void memory_destroy() {
   if (swrom) {
      free(swrom);
   }
   if (memory) {
      free(memory);
   }
}

void memory_set_io_window(int low, int high) {
   io_low = low;
   io_high = high;
}

void memory_set_tube_window(int low, int high) {
   tube_low = low;
   tube_high = high;
}

void memory_set_rom_latch_addr(int addr) {
   rom_latch_addr = addr;
   swrom = malloc(SWROM_NUM_BANKS * SWROM_SIZE * sizeof(int));
   for (int i = 0; i < SWROM_NUM_BANKS * SWROM_SIZE; i++) {
      swrom[i] = -1;
   }
}

void memory_set_modelling(int bitmask) {
   mem_model = bitmask;
}

void memory_set_rd_logging(int bitmask) {
   mem_rd_logging = bitmask;
}

void memory_set_wr_logging(int bitmask) {
   mem_wr_logging = bitmask;
}

void memory_read(int data, int ea, mem_access_t type) {
   assert(ea >= 0);
   assert(data >= 0);
   if (mem_rd_logging & (1 << type)) {
      char *bp = buffer;
      bp += write_s(bp, "memory  read: ");
      bp += write_addr(bp, ea);
      bp += write_s(bp, " = ");
      write_hex2(bp, data);
      bp += 2;
      *bp++ = 0;
      puts(buffer);
   }
   if (mem_model & (1 << type)) {
      if (ea < io_low || ea >= io_high) {
         int *memptr = get_memptr(ea);
         if (*memptr >=0 && *memptr != data) {
            char *bp = buffer;
            bp += write_s(bp, "memory modelling failed at ");
            bp += write_addr(bp, ea);
            bp += write_s(bp, ": expected ");
            write_hex2(bp, *memptr);
            bp += 2;
            bp += write_s(bp, " actual ");
            write_hex2(bp, data);
            bp += 2;
            *bp++ = 0;
            puts(buffer);
            failflag |= 1;
         }
         *memptr = data;
      }
   }
   // Pass on to tube decoding
   if (ea >= tube_low && ea <= tube_high) {
      tube_read(ea & 7, data);
   }
}

void memory_write(int data, int ea, mem_access_t type) {
   assert(ea >= 0);
   assert(data >= 0);
   if (mem_wr_logging & (1 << type)) {
      char *bp = buffer;
      bp += write_s(bp, "memory write: ");
      bp += write_addr(bp, ea);
      bp += write_s(bp, " = ");
      write_hex2(bp, data);
      bp += 2;
      *bp++ = 0;
      puts(buffer);
   }
   // TODO: chack this...Data can be negative, which means the memory becomes undefined again
   if (mem_model & (1 << type)) {
      int *memptr = get_memptr(ea);
      *memptr = data;
   }
   // Pass on to tube decoding
   if (ea >= tube_low && ea <= tube_high) {
      tube_write(ea & 7, data);
   }
   // Update the ROM latch
   if (ea == rom_latch_addr) {
      rom_latch_data = (data >= 0) ? (data & 15) : -1;
   }
}

int memory_read_raw(int ea) {
   return *get_memptr(ea);
}
