#!/bin/bash

cd firmware
make clobber
make
cd ..

make clean
make
