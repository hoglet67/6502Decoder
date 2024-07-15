#ifndef _SYMBOLS_H

#define _SYMBOLS_H

void symbol_init(int size);

void symbol_add(char *name, int address);

char *symbol_lookup(int address);

void symbol_import_swift(char *filename);

#endif
