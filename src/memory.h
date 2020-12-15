#ifndef MEMORY_H
#define MEMORY_H

typedef enum {
   MEM_INSTR    = 0,
   MEM_POINTER  = 1,
   MEM_DATA     = 2,
   MEM_STACK    = 3,
} mem_access_t;

void memory_init(int size);


void memory_set_modelling(int bitmask);

void memory_set_rd_logging(int bitmask);

void memory_set_wr_logging(int bitmask);

void memory_set_io_window(int low, int high);

void memory_set_tube_window(int low, int high);

void memory_set_rom_latch_addr(int addr);

void memory_read(int data, int ea, mem_access_t type);

void memory_write(int data, int ea, mem_access_t type);

int memory_read_raw(int ea);

void memory_destroy();

#endif
