#!/bin/bash

gcc -Wall -O3 -D_GNU_SOURCE -o decode6502 src/main.c src/em_6502.c src/profiler.c
