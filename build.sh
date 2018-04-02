#!/bin/bash

gcc -Wall -O3 -o decode6502 src/main.c src/em_6502.c src/profiler.c
