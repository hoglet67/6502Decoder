#!/bin/bash

cd firmware
make clobber
make
cd ..

./configure
make clean
make
