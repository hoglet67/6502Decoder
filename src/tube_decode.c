#include <stdio.h>
#include <inttypes.h>

// #define DEBUG

enum R1_enum {
   R1_IDLE,
   R1_EVENT_0,
   R1_EVENT_1,
   R1_EVENT_2
};

enum R2_enum {
   R2_IDLE,
   R2_OSCLI_0,
   R2_OSBYTELO_0,
   R2_OSBYTELO_1,
   R2_OSBYTEHI_0,
   R2_OSBYTEHI_1,
   R2_OSBYTEHI_2,
   R2_OSWORD_0,
   R2_OSWORD_1,
   R2_OSWORD_2,
   R2_OSWORD_3,
   R2_OSWORD0_0,
   R2_OSWORD0_1,
   R2_OSWORD0_2,
   R2_OSWORD0_3,
   R2_OSWORD0_4,
   R2_OSWORD_FB_0,
   R2_OSWORD_FB_1,
   R2_OSWORD_FB_2,
   R2_OSWORD_FB_IO,
   R2_OSWORD_FB_FDC,
   R2_OSWORD_FF_0,
   R2_OSWORD_FF_1,
   R2_OSWORD_FF_2,
   R2_OSWORD_FF_3,
   R2_OSWORD_FF_4,
   R2_OSWORD_FF_5,
   R2_OSARGS_0,
   R2_OSARGS_1,
   R2_OSARGS_2,
   R2_OSARGS_3,
   R2_OSBGET_0,
   R2_OSBPUT_0,
   R2_OSFIND_0,
   R2_OSFIND_1,
   R2_OSFIND_2,
   R2_OSFILE_0,
   R2_OSFILE_1,
   R2_OSFILE_2,
   R2_OSGBPB_0
};

enum R4_enum {
   R4_IDLE,
   R4_ERROR_0,
   R4_XFER_0,
   R4_XFER_1,
   R4_XFER_2,
   R4_XFER_3,
   R4_XFER_4,
   R4_XFER_5,
   R4_XFER_6
};

static int in_error = -1;

static void print_call(char *call, int a, int x, int y, uint8_t *name, uint8_t *block, int block_len) {
   int i;
   printf("%s: ", call);
   if (a >= 0) {
      printf("A=%02x ", a);
   }
   if (x >= 0) {
      printf("X=%02x ", x);
   }
   if (y >= 0) {
      printf("Y=%02x ", y);
   }
   if (name) {
      printf("STRING=%s ", name);
   }
   if (block && block_len > 0) {
      printf("BLOCK=");
      for (i = 0; i < block_len; i++) {
         printf("%02x ", block[i]);
      }
   }
   printf("\n");
}


void r1_h2p_state_machine(uint8_t data) {
   static int a = -1;
   static int x = -1;
   static int y = -1;
   static int state = R1_IDLE;

#ifdef DEBUG
   printf("tube write: R1 = %02x\n", data);
#endif

   switch (state) {
   case R1_IDLE:
      if (data & 0x80) {
         printf("R1: Escape: flag=%02x\n", data);
      } else {
         state = R1_EVENT_0;
      }
      break;
   case R1_EVENT_0:
      y = data;
      state = R1_EVENT_1;
      break;
   case R1_EVENT_1:
      x = data;
      state = R1_EVENT_2;
      break;
   case R1_EVENT_2:
      a = data;
      printf("R1: Event: A=%02x X=%02x Y=%02x\n", a, x, y);
      state = R1_IDLE;
      break;
   }
}

void r2_h2p_state_machine(uint8_t data) {
   static uint8_t error[512];
#ifdef DEBUG
   printf("tube write: R2 = %02x\n", data);
#endif
   if (in_error >= 0) {
      // Error    R4: &FF R2: &00 err string &00
      error[in_error] = data;
      if (data == 0 && in_error > 0) {
         printf("R4/R2: Error num=%d message=%s\n", error[1], error + 2);
         in_error = -1;
      } else {
         in_error++;
      }
      return;
   }
}

void r4_h2p_state_machine(uint8_t data) {
   static int    action = -1;
   static int        id = -1;
   static uint32_t addr = 0;
   static int      sync = -1;
   static int     state = R4_IDLE;

#ifdef DEBUG
   printf("tube write: R4 = %02x\n", data);
#endif

   switch (state) {
   case R4_IDLE:
      if (data == 0xff) {
         in_error = 0;
      } else if (data < 0x08) {
         action = data;
         state = R4_XFER_0;
      } else {
         printf("R4: illegal transfer type: %02x\n", data);
      }
      break;
   case R4_XFER_0:
      id = data;
      if (action == 5) {
      printf("R4: Transfer: Action=%02x ID=%02x\n", action, id);
         state = R4_IDLE;
      } else {
         state = R4_XFER_1;
      }
      break;
   case R4_XFER_1:
      addr = data;
      state = R4_XFER_2;
      break;
   case R4_XFER_2:
      addr <<= 8;
      addr |= data;
      state = R4_XFER_3;
      break;
   case R4_XFER_3:
      addr <<= 8;
      addr |= data;
      state = R4_XFER_4;
      break;
   case R4_XFER_4:
      addr <<= 8;
      addr |= data;
      state = R4_XFER_5;
      break;
   case R4_XFER_5:
      sync = data;
      printf("R4: Transfer: Action=%02x ID=%02x Addr=%08x Sync=%02x\n", action, id, addr, sync);
      state = R4_IDLE;
      break;
   }
}

void r2_p2h_state_machine(uint8_t data) {
   int i;
   static int a = -1;
   static int x = -1;
   static int y = -1;
   static int in_length = -1;
   static int state = R2_IDLE;
   static uint8_t buffer[512];
   static int index = 0;

#ifdef DEBUG
   printf("tube read:  R2 = %02x\n", data);
#endif

   // There seems to be a spurious read of R2 by the host
   // during error handling, which we supress here.
   if (in_error == 0) {
      return;
   }

   // Save the data
   buffer[index] = data;
   if (index < sizeof(buffer) - 1) {
      index++;
   } else {
      printf("Buffer overflow!, state = %d\n", state);
   }
   switch (state) {
   case R2_IDLE:
      switch (data) {
      case 0x00:
         print_call("R2: OSRDCH", -1, -1, -1, NULL, NULL, -1);
         break;
      case 0x02:
         state = R2_OSCLI_0;
         break;
      case 0x04:
         state = R2_OSBYTELO_0;
         break;
      case 0x06:
         state = R2_OSBYTEHI_0;
         break;
      case 0x08:
         state = R2_OSWORD_0;
         break;
      case 0x0A:
         state = R2_OSWORD0_0;
         break;
      case 0x0C:
         state = R2_OSARGS_0;
         break;
      case 0x0E:
         state = R2_OSBGET_0;
         break;
      case 0x10:
         state = R2_OSBPUT_0;
         break;
      case 0x12:
         state = R2_OSFIND_0;
         break;
      case 0x14:
         state = R2_OSFILE_0;
         break;
      case 0x16:
         state = R2_OSGBPB_0;
         break;
      default:
         printf("Illegal R2 tube command %02x\n", data);
      }
      break;


   case R2_OSCLI_0:
      if (data == 0x0D) {
         buffer[index - 1] = 0;
         print_call("R2: OSCLI", -1, -1, -1, buffer + 1, NULL, -1);
         state = R2_IDLE;
      }
      break;

   case R2_OSBYTELO_0:
      x = data;
      state = R2_OSBYTELO_1;
      break;
   case R2_OSBYTELO_1:
      a = data;
      print_call("R2: OSBYTE", a, x, -1, NULL, NULL, -1);
      state = R2_IDLE;
      break;

   case R2_OSBYTEHI_0:
      x = data;
      state = R2_OSBYTEHI_1;
      break;
   case R2_OSBYTEHI_1:
      y = data;
      state = R2_OSBYTEHI_2;
      break;
   case R2_OSBYTEHI_2:
      a = data;
      print_call("R2: OSBYTE", a, x, y, NULL, NULL, -1);
      state = R2_IDLE;
      break;


   case R2_OSWORD_0:
      a = data;
      if (a == 0xfb) {
         state = R2_OSWORD_FB_0;
      } else if (a == 0xff) {
         state = R2_OSWORD_FF_0;
      } else {
         state = R2_OSWORD_1;
      }
      break;
   case R2_OSWORD_1:
      if (a == 0xfc) {
         // This seems like a violation of the protocol!
         // tube read:  3 = 08
         // tube read:  3 = fc
         // tube read:  3 = 00 // in_length = 0
         // tube read:  3 = fe
         // tube read:  3 = ff
         // tube read:  3 = 00 // out_length = 0
         in_length = 2;
      } else {
         in_length = data;
      }
      state = R2_OSWORD_2;
      break;
   case R2_OSWORD_2:
      if (index == in_length + 3) {
         state = R2_OSWORD_3;
      }
      break;
   case R2_OSWORD_3:
      // ignore outlen for now
      print_call("R2: OSWORD", a, -1, -1, NULL, buffer + 3, in_length);
      state = R2_IDLE;
      break;


   case R2_OSWORD0_0:
      state = R2_OSWORD0_1;
      break;
   case R2_OSWORD0_1:
      state = R2_OSWORD0_2;
      break;
   case R2_OSWORD0_2:
      state = R2_OSWORD0_3;
      break;
   case R2_OSWORD0_3:
      state = R2_OSWORD0_4;
      break;
   case R2_OSWORD0_4:
      print_call("R2: OSWORD0", -1, -1, -1, NULL, buffer + 1, 5);
      state = R2_IDLE;
      break;


   case R2_OSWORD_FB_0:
      // 08 FB 00
      // consume the 00
      state = R2_OSWORD_FB_1;
      break;

   case R2_OSWORD_FB_1:
      // 08 FB 00 <cmd>
      // consume the cmd
      switch (data) {
      case 0:
         // 0 indicates finish
         state = R2_OSWORD_FB_2;
         break;
      case 1:
      case 2:
         // 1 indicates master FDC command
         // 2 indicated beeb FDC command
         index = 0;
         state = R2_OSWORD_FB_FDC;
         break;
      case 3:
         // 3 indicates claim tube
         printf("OSWORD: A=fb: tube claim\n");
         break;
      case 4:
         // 4 indicates release tube
         printf("OSWORD: A=fb: tube release\n");
         break;
      default:
         // anything else indicates &FExx write
         x = data;
         state = R2_OSWORD_FB_IO;
      }
      break;
   case R2_OSWORD_FB_2:
      state = R2_IDLE;
      break;

   case R2_OSWORD_FB_FDC:
      if (index == 9) {
         printf("OSWORD: A=fb: fdc disk command: ");
         for (i = 8; i >= 0; i--) {
            printf("%02x ", buffer[i]);
         }
         printf("(");
         switch (data >> 4) {
         case 0:
            printf("Restore");
            break;
         case 1:
            printf("Seek");
            break;
         case 2:
         case 3:
            printf("Step");
            break;
         case 4:
         case 5:
            printf("Step in");
            break;
         case 6:
         case 7:
            printf("Step out");
            break;
         case 8:
         case 9:
            printf("Read sector");
            break;
         case 10:
         case 11:
            printf("Write sector");
            break;
         case 12:
            printf("Read address");
            break;
         case 13:
            printf("Read track");
            break;
         case 14:
            printf("Write track");
            break;
         case 15:
            printf("Force interrupt");
            break;
         }
         printf(")\n");
         state = R2_OSWORD_FB_1;
      }
      break;

   case R2_OSWORD_FB_IO:
      if (x == 0x24) {
         printf("OSWORD: A=fb: fdc disk control %02x\n", data);
      } else if (x == 0x29) {
         printf("OSWORD: A=fb: fdc set track %d\n", data);
      } else if (x == 0x2a) {
         printf("OSWORD: A=fb: fdc set sector %d\n", data);
      } else if (x == 0x2b) {
         printf("OSWORD: A=fb: fdc set data %d\n", data);
      } else {
         printf("OSWORD: A=fb: io write FE%02X=%02X\n", x, data);
      }
      state = R2_OSWORD_FB_1;
      break;

   case R2_OSWORD_FF_0:
      //       VV
      // 08 FF 00 [ <addr hi> <addr lo> [ <sync> 00 ]...  <sync> FF ] ...
      // terminated with an <addr hi> of zero
      state = R2_OSWORD_FF_1;
      break;

   case R2_OSWORD_FF_1:
      // <addr hi> of zero terminates
      if (data == 0) {
         state = R2_OSWORD_FF_5;
      } else {
         state = R2_OSWORD_FF_2;
      }
      break;

   case R2_OSWORD_FF_2:
      // <addr lo>
      state = R2_OSWORD_FF_3;
      break;

   case R2_OSWORD_FF_3:
      // <sync>
      state = R2_OSWORD_FF_4;
      break;

   case R2_OSWORD_FF_4:
      if (data == 0x00) {
         state = R2_OSWORD_FF_3;
      } else if (data == 0xFF) {
         print_call("R2: OSWORD", a, -1, -1, NULL, buffer + 3, index - 3);
         index = 3;
         state = R2_OSWORD_FF_1;
      } else {
         printf("Osword FF protocol violation\n");
         state = R2_IDLE;
      }
      break;

   case R2_OSWORD_FF_5:
      print_call("R2: OSWORD", a, -1, -1, NULL, buffer + 3, 3);
      state = R2_IDLE;
      break;

   case R2_OSARGS_0:
      y = data;
      state = R2_OSARGS_1;
      break;
   case R2_OSARGS_1:
      if (index == 6) {
         // &0C Y <4-byte block> A
         state = R2_OSARGS_2;
      }
      break;
   case R2_OSARGS_2:
      a = data;
      state = R2_IDLE;
      print_call("R2: OSARGS", a, -1, y, NULL, buffer + 2, 5);
      break;


   case R2_OSBGET_0:
      printf("R2: OSBGET not yet implemented\n");
      state = R2_IDLE;
      break;
   case R2_OSBPUT_0:
      printf("R2: OSBPUT not yet implemented\n");
      state = R2_IDLE;
      break;


   case R2_OSFIND_0:
      // OSFIND   R2: &12 &00 Y
      // OSFIND   R2: &12 A string &0D
      a = data;
      if (a == 0) {
         state = R2_OSFIND_1;
      } else {
         state = R2_OSFIND_2;
      }
      break;
   case R2_OSFIND_1:
      y = data;
      print_call("R2: OSFIND", a, -1, y, NULL, NULL, -1);
      state = R2_IDLE;
      break;
   case R2_OSFIND_2:
      if (data == 0x0d) {
         buffer[index - 1] = 0;
         print_call("R2: OSFIND", a, -1, -1, buffer + 2, NULL, -1);
         state = R2_IDLE;
      }
      break;

   case R2_OSFILE_0:
      // OSFILE   R2: &14 block string &0D A
      if (index == 17) {
         state = R2_OSFILE_1;
      }
      break;
   case R2_OSFILE_1:
      if (data == 0x0d) {
         buffer[index - 1] = 0;
         state = R2_OSFIND_2;
      }
      break;
   case R2_OSFILE_2:
      a = data;
      print_call("R2: OSFILE", a, -1, -1, buffer + 16, buffer + 1, 16);
      state = R2_IDLE;
      break;


   case R2_OSGBPB_0:
      printf("R2: OSGBPB not yet implemented\n");
      state = R2_IDLE;
      break;

   default:
      break;
   }
   if (state == R2_IDLE) {
      index = 0;
   }
}


// Parasite Initiated Requests
void tube_read(int reg, uint8_t data) {
   if (reg == 1) {
      printf("OSWRCH: %c <%02x>\n", (data >= 32 && data < 127) ? data : '.', data);
   }
   if (reg == 3) {
      r2_p2h_state_machine(data);
   }
}

// Host Initiated Requests
void tube_write(int reg, uint8_t data) {
   if (reg == 1) {
      r1_h2p_state_machine(data);
   }
   if (reg == 3) {
      r2_h2p_state_machine(data);
   }
   if (reg == 7) {
      r4_h2p_state_machine(data);
   }
}
