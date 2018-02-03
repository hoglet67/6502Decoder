#!/bin/bash

declare -a machine_names

machine_names=(
    master
    beeb
    elk
    beebr65c02
)

for machine in "${machine_names[@]}"
do
   rm -f ${machine}/*.tmp
   rm -f ${machine}/*.log
done
