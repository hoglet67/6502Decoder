#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "defs.h"
#include "tube_decode.h"
#include "memory.h"

// Sideways ROM

#define SWROM_SIZE          0x4000
#define SWROM_NUM_BANKS     16

// Standard sideways ROM (upto 16 banks)
static int *swrom         = NULL;
static int rom_latch      = 0;

// Extra Master registers
static int acccon_latch   = 0;
static int *lynne;           // 20KB overlaid at 3000-7FFF
static int *hazel;           //  8KB overlaid at C000-DFFF
static int *andy;            //  4KB overlaid at 8000-8FFF
static int vdu_op;           // the last instruction fetch was by the VDU driver

// Main Memory

static int *memory        = NULL;
static int mem_model      = 0;
static int mem_rd_logging = 0;
static int mem_wr_logging = 0;
static int addr_digits    = 0;

// IO
static int tube_low       = -1;
static int tube_high      = -1;

static char buffer[256];

// Machine specific memory rd/wr handlers
static void (*memory_read_fn)(int data, int ea);
static int (*memory_write_fn)(int data, int ea);

// Pre-calculate a label for each 4K page in memory
// These are manipulated as the ROM and ACCCON latches are modified
static char bank_id[32];

#define TO_HEX(value) ((value) + ((value) < 10 ? '0' : 'A' - 10))

int write_bankid(char *bp, int ea) {
  if (ea < 0 || ea >= 0x10000) {
      *bp++ = ' ';
      *bp++ = ' ';
   } else {
      char *bid = bank_id + ((ea & 0xF000) >> 11);
      *bp++ = *bid++;
      *bp++ = *bid++;
   }
  return 2;
}

static inline int write_addr(char *bp, int ea) {
   bp += write_bankid(bp, ea);
   int shift = (addr_digits - 1) << 2; // 6 => 20
   for (int i = 0; i < addr_digits; i++) {
      int value = (ea >> shift) & 0xf;
      *bp++ = TO_HEX(value);
      shift -= 4;
   }
   return addr_digits + 2;
}


static inline void log_memory_access(char *msg, int data, int ea, int ignored) {
   char *bp = buffer;
   bp += write_s(bp, msg);
   bp += write_addr(bp, ea);
   bp += write_s(bp, " = ");
   write_hex2(bp, data);
   bp += 2;
   if (ignored) {
   bp += write_s(bp, " (ignored)");
   }
   *bp++ = 0;
   puts(buffer);
}


static inline void log_memory_fail(int ea, int expected, int actual) {
   char *bp = buffer;
   bp += write_s(bp, "memory modelling failed at ");
   bp += write_addr(bp, ea);
   bp += write_s(bp, ": expected ");
   write_hex2(bp, expected);
   bp += 2;
   bp += write_s(bp, " actual ");
   write_hex2(bp, actual);
   bp += 2;
   *bp++ = 0;
   puts(buffer);
}

static void set_tube_window(int low, int high) {
   tube_low = low;
   tube_high = high;
}

static int *init_ram(int size) {
   int *ram =  malloc(size * sizeof(int));
   for (int i = 0; i < size; i++) {
      ram[i] = -1;
   }
   return ram;
}


static void set_rom_latch(int data) {
   rom_latch = data;
   // Update the bank id string
   char *bid = bank_id + 16; // 8xxx
   char c = TO_HEX(data & 0xf);
   if (data & 0x80) {
      // Andy RAM is paged in to &8000-&8FFF
      *bid++ = 'R';
   } else {
      *bid++ = c;
   }
   *bid++ = ':';
   *bid++ = c;
   *bid++ = ':';
   *bid++ = c;
   *bid++ = ':';
   *bid++ = c;
   *bid++ = ':';
}

static void set_acccon_latch(int data) {
   char *bid;
   acccon_latch = data;
   // Update the bank id string for Lynnn (Shadow RAM) based on bit 2
   // TODO: this is not sufficient; needs to take account of vdu_op which changes each instruction
   bid = bank_id + 6; // 3xxx
   for (int i = 0; i < 5; i++) {
      if (data & 0x04) {
         // Shadow RAM is paged into &3000-7FFF
         *bid++ = 'S';
         *bid++ = ':';
      } else {
         // Normal RAM is paged into &3000-7FFF
         *bid++ = ' ';
         *bid++ = ' ';
      }
   }
   // Update the bank id string Hazel (MOS Overlay) based on bit 3
   bid = bank_id + 24; // Cxxx
   for (int i = 0; i < 2; i++) {
      if (data & 0x08) {
         // Hazel RAM is paged into &C000-&DFFF
         *bid++ = 'H';
         *bid++ = ':';
      } else {
         // OS is pages into &C000-&DFFF
         *bid++ = ' ';
         *bid++ = ' ';
      }
   }

}

// ==================================================
// Beeb Memory Handlers
// ==================================================

static inline int *get_memptr_beeb(int ea) {
   if (ea >= 0x8000 && ea < 0xC000) {
      return swrom + (rom_latch << 14) + (ea & 0x3FFF);
   } else {
      return memory + ea;
   }
}

static void memory_read_beeb(int data, int ea) {
   if (ea < 0xfc00 || ea >= 0xff00) {
      int *memptr = get_memptr_beeb(ea);
      if (*memptr >=0 && *memptr != data) {
         log_memory_fail(ea, *memptr, data);
         failflag |= 1;
      }
      *memptr = data;
   }
}

static int memory_write_beeb(int data, int ea) {
   if (ea == 0xfe30) {
      set_rom_latch(data & 0xf);
   }
   int *memptr = get_memptr_beeb(ea);
   *memptr = data;
   return 0;
}

static void init_beeb(int logtube) {
   swrom = init_ram(SWROM_NUM_BANKS * SWROM_SIZE);
   memory_read_fn  = memory_read_beeb;
   memory_write_fn = memory_write_beeb;
   if (logtube) {
      set_tube_window(0xfee0, 0xfee8);
   }
}

// ==================================================
// Master Memory Handlers
// ==================================================

static inline int *get_memptr_master(int ea) {
   if ((acccon_latch & 0x08) && ea >= 0xc000 && ea < 0xe000) {
      return hazel + (ea & 0x1FFF);
   } else if ((rom_latch & 0x80) && ea >= 0x8000 && ea < 0x9000) {
      return andy + (ea & 0x0FFF);
   } else if (ea >= 0x3000 && ea < 0x8000 && (acccon_latch & (vdu_op ? 0x02 : 0x04))) {
      return lynne + (ea - 0x3000);
   } else if (ea >= 0x8000 && ea < 0xC000) {
      return swrom + ((rom_latch & 0xf) << 14) + (ea & 0x3FFF);
   } else {
      return memory + ea;
   }
}

static void memory_read_master(int data, int ea) {
   if (ea < 0xfc00 || ea >= 0xff00) {
      int *memptr = get_memptr_master(ea);
      if (*memptr >=0 && *memptr != data) {
         log_memory_fail(ea, *memptr, data);
         failflag |= 1;
      }
      *memptr = data;
   }
}

static int memory_write_master(int data, int ea) {
   if (ea == 0xfe30) {
      set_rom_latch(data & 0x8f);
   }
   if (ea == 0xfe34) {
      set_acccon_latch(data & 0xff);
   }
   // Determine if the access is writeable
   if ((ea < 0x8000) ||
       (ea < 0x9000 && (rom_latch & 0x80)) ||
       (ea < 0xc000 && ((rom_latch & 0x0c) == 0x04)) ||
       (ea >= 0xc000 && ea < 0xe000 && (acccon_latch & 0x08)) ||
       (ea >= 0xfc00 && ea < 0xff00)) {
      int *memptr = get_memptr_master(ea);
      *memptr = data;
      return 0;
   } else {
      return 1;
   }
}

static void init_master(int logtube) {
   swrom = init_ram(SWROM_NUM_BANKS * SWROM_SIZE);
   lynne = init_ram(20480); // 20KB overlaid at 3000-7FFF
   hazel = init_ram(8192);  //  8KB overlaid at C000-DFFF
   andy  = init_ram(4096);  //  4KB overlaid at 8000-8FFF
   memory_read_fn  = memory_read_master;
   memory_write_fn = memory_write_master;
   if (logtube) {
      set_tube_window(0xfee0, 0xfee8);
   }
}

// ==================================================
// Elk Memory Handlers
// ==================================================

static inline int *get_memptr_elk(int ea) {
   if (ea >= 0x8000 && ea < 0xC000) {
      return swrom + (rom_latch << 14) + (ea & 0x3FFF);
   } else {
      return memory + ea;
   }
}

static void memory_read_elk(int data, int ea) {
   if (ea < 0xfc00 || ea >= 0xff00) {
      int *memptr = get_memptr_elk(ea);
      if (*memptr >=0 && *memptr != data) {
         log_memory_fail(ea, *memptr, data);
         failflag |= 1;
      }
      *memptr = data;
   }
}

static int memory_write_elk(int data, int ea) {
   if (ea == 0xfe05) {
      set_rom_latch(data & 0xf);
   }
   int *memptr = get_memptr_elk(ea);
   *memptr = data;
   return 0;
}

static void init_elk(int logtube) {
   swrom = init_ram(SWROM_NUM_BANKS * SWROM_SIZE);
   memory_read_fn  = memory_read_elk;
   memory_write_fn = memory_write_elk;
   if (logtube) {
      set_tube_window(0xfce0, 0xfce8);
   }
}

// ==================================================
// Default Memory Handlers
// ==================================================

static void memory_read_default(int data, int ea) {
   if (memory[ea] >= 0 && memory[ea] != data) {
      log_memory_fail(ea,memory[ea], data);
      failflag |= 1;
   }
   memory[ea] = data;
}

static int memory_write_default(int data, int ea) {
   memory[ea] = data;
   return 0;
}

static void init_default(int logtube) {
   memory_read_fn  = memory_read_default;
   memory_write_fn = memory_write_default;
}

// ==================================================
// Public Methods
// ==================================================

void memory_init(int size, machine_t machine, int logtube) {
   memory = init_ram(size);
   // Setup the machine specific memory read/write handler
   switch (machine) {
   case MACHINE_BEEB:
      init_beeb(logtube);
      break;
   case MACHINE_MASTER:
      init_master(logtube);
      break;
   case MACHINE_ELK:
      init_elk(logtube);
      break;
   default:
      init_default(logtube);
      break;
   }
   // Calculate the number of digits to represent an address
   addr_digits = 0;
   size--;
   while (size) {
      size >>= 1;
      addr_digits++;
   }
   addr_digits = (addr_digits + 3) >> 2;
   // Initialize bank labels (2 chars per 4K page)
   for (int i = 0; i < 32; i++) {
      bank_id[i] = ' ';
   }
}

void memory_destroy() {
   if (swrom) {
      free(swrom);
   }
   if (memory) {
      free(memory);
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
   // Update the vdu_op state every fetch (used by the master only)
   if (type == MEM_FETCH) {
      vdu_op = ((acccon_latch & 0x08) == 0x00) && ((ea & 0xffe000) == 0xc000);
      type = MEM_INSTR;
   }
   // Log memory read
   if (mem_rd_logging & (1 << type)) {
      log_memory_access("Rd: ", data, ea, 0);
   }
   // Delegate memory read to machine specific handler
   if (mem_model & (1 << type)) {
      (*memory_read_fn)(data, ea);
   }
   // Pass on to tube decoding
   if (ea >= tube_low && ea <= tube_high) {
      tube_read(ea & 7, data);
   }
}

void memory_write(int data, int ea, mem_access_t type) {
   assert(ea >= 0);
   assert(data >= 0);
   // Delegate memory write to machine specific handler
   int ignored = 0;
   if (mem_model & (1 << type)) {
      ignored = (*memory_write_fn)(data, ea);
   }
   // Log memory write
   if (mem_wr_logging & (1 << type)) {
      log_memory_access("Wr: ", data, ea, ignored);
   }
   // Pass on to tube decoding
   if (ea >= tube_low && ea <= tube_high) {
      tube_write(ea & 7, data);
   }
}

int memory_read_raw(int ea) {
   return memory[ea];
}
