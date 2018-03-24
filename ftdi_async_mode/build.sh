#!/bin/bash
gcc -I /usr/include/libftdi1 -I /usr/include/libusb-1.0/ stream_test.c -o stream_test -lftdi1 -lusb-1.0
