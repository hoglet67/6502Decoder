#!/bin/bash

DECODE=../../decode6502

declare -A machine_options
machine_options[master]="--machine=master -c --vecrst=A9E364"
#machine_options[beeb]="--vecrst=A9D9CD"
#machine_options[electon]="--vecrst=A9D8D2"

declare -A test_options

test_options[ref]="--phi2="

test_options[sync_nornw]="--phi2= --rnw="
test_options[sync_norst]="--phi2= --rst="
test_options[sync_nordy]="--phi2= --rdy="
test_options[sync_nornw_norst]="--phi2= --rnw= --rst="
test_options[sync_nornw_nordy]="--phi2= --rnw= --rdy="
test_options[sync_norst_nordy]="--phi2= --rst= --rdy="
test_options[sync_nornw_norst_nordy]="--phi2= --rnw= --rst= --rdy="

test_options[syncless]="--phi2= --sync="
test_options[syncless_nornw]="--phi2= --sync= --rnw="
test_options[syncless_norst]="--phi2= --sync= --rst="
test_options[syncless_nordy]="--phi2= --sync= --rdy="
test_options[syncless_nornw_norst]="--phi2= --sync= --rnw= --rst="
test_options[syncless_nornw_nordy]="--phi2= --sync= --rnw= --rdy="
test_options[syncless_norst_nordy]="--phi2= --sync= --rst= --rdy="
test_options[syncless_nornw_norst_nordy]="--phi2= --sync= --rnw= --rst= --rdy="

for machine in "${!machine_options[@]}"
do
    pushd ${machine}
    for data in reset
    do
        # First, generate all the data for the test cases
        echo "Running decoder tests"
        for test in "${!test_options[@]}"
        do
            log=trace_${data}_${test}.log
            gunzip < ${data}.bin.gz | ${DECODE} ${machine_options[${machine}]} ${test_options[${test}]} > ${log}

            # If the file contains a RESET marker, prune any lines before this
            if grep -q RESET ${log}; then
                sed -n '/RESET/,$p' < ${log} > tmp
                mv tmp ${log}
            fi

            fail_count=`grep fail ${log} | wc -l`
            md5=`md5sum ${log} | cut -c1-8`
            printf "Machine: %s; Data: %s; Test: %28s; MD5: %s; Fail Count: %s\n" ${machine} ${data} ${test} ${md5} ${fail_count}
            echo "  gunzip < ${data}.bin.gz | ${DECODE} ${machine_options[${machine}]} ${test_options[${test}]}"
        done
        echo "Checking decoder results"
        # Next, compare the tests against the reference
        for test in "${!test_options[@]}"
        do
            log=trace_${data}_${test}.log
            ref=trace_${data}_ref.log
            diff_count=`diff ${ref} ${log} | wc -l`
            printf "Machine: %s; Data: %s; Test: %28s; Diff Count: %s\n" ${machine} ${data} ${test} ${diff_count}
            diff ${ref} ${log}
        done
    done
    popd
done
