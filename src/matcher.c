#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#define N 24

#define SIZE (1<<N)
#define MASK (SIZE-1)

uint8_t buffer[SIZE];
uint8_t pattern[] = { 0x4c, 0x83, 0x90 };

int main(int argc, char *argv[]) {

   uint32_t ptr = 0;
   int c = 0;
   memset(buffer, 0xff, sizeof(buffer));
   uint64_t i = 0;
   while ((c = getchar()) != EOF) {
      uint32_t tmp = ptr;
      buffer[ptr] = (uint8_t) c;
      ptr = (ptr + 1) & MASK;
      i++;
      unsigned int n;
      uint8_t *pp = pattern + sizeof(pattern) - 1;
      for (n = 0; n < sizeof(pattern) && buffer[tmp] == *pp; n++) {
         pp--;
         tmp = (tmp - 2) & MASK;
      }
      if (n == sizeof(pattern)) {
         fprintf(stderr, "Matched pattern at %ld!!\n", i);
         break;
      }
   }
   // Move on one more byte, so we'll end up with correct 16-bit alignment
   ptr = (ptr + 1) & MASK;
   for (i = ptr; i < SIZE; i++) {
      putchar(buffer[i]);
   }
   for (i = 0; i < ptr; i++) {
      putchar(buffer[i]);
   }

   return 0;
}
