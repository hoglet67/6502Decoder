#!/bin/bash

DECODE=../../decode6502

common_options="--phi2= -h -s"

declare -A machine_options
machine_options[master]="--machine=master -c --vecrst=A9E364"
#machine_options[beeb]="--vecrst=A9D9CD"
#machine_options[electon]="--vecrst=A9D8D2"

declare -a test_names
test_names[0]=sync
test_names[1]=sync_nornw
test_names[2]=sync_norst
test_names[3]=sync_nordy
test_names[4]=sync_nornw_norst
test_names[5]=sync_nornw_nordy
test_names[6]=sync_norst_nordy
test_names[7]=sync_nornw_norst_nordy
test_names[8]=nosync
test_names[9]=nosync_nornw
test_names[10]=nosync_norst
test_names[11]=nosync_nordy
test_names[12]=nosync_nornw_norst
test_names[13]=nosync_nornw_nordy
test_names[14]=nosync_norst_nordy
test_names[15]=nosync_nornw_norst_nordy


declare -A test_options
test_options[sync]=""
test_options[sync_nornw]="--rnw="
test_options[sync_norst]="--rst="
test_options[sync_nordy]="--rdy="
test_options[sync_nornw_norst]="--rnw= --rst="
test_options[sync_nornw_nordy]="--rnw= --rdy="
test_options[sync_norst_nordy]="--rst= --rdy="
test_options[sync_nornw_norst_nordy]="--rnw= --rst= --rdy="
test_options[nosync]="--sync="
test_options[nosync_nornw]="--sync= --rnw="
test_options[nosync_norst]="--sync= --rst="
test_options[nosync_nordy]="--sync= --rdy="
test_options[nosync_nornw_norst]="--sync= --rnw= --rst="
test_options[nosync_nornw_nordy]="--sync= --rnw= --rdy="
test_options[nosync_norst_nordy]="--sync= --rst= --rdy="
test_options[nosync_nornw_norst_nordy]="--sync= --rnw= --rst= --rdy="

# Use the sync based decoder as the deference
ref=sync

for machine in "${!machine_options[@]}"
do
    pushd ${machine}
    for data in reset
    do
        # First, generate all the data for the test cases
        echo "Running decoder tests"
        for test in "${test_names[@]}"
        do
            log=trace_${data}_${test}.log
            gunzip < ${data}.bin.gz | ${DECODE} ${common_options} ${machine_options[${machine}]} ${test_options[${test}]} > ${log}

            # If the file contains a RESET marker, prune any lines before this
            if grep -q RESET ${log}; then
                sed -n '/RESET/,$p' < ${log} > tmp
                mv tmp ${log}
            fi

            fail_count=`grep fail ${log} | wc -l`
            md5=`md5sum ${log} | cut -c1-8`
            printf "Machine: %s; Data: %s; Test: %28s; MD5: %s; Fail Count: %s\n" ${machine} ${data} ${test} ${md5} ${fail_count}
            # echo "  gunzip < ${data}.bin.gz | ${DECODE} ${common_options} ${machine_options[${machine}]} ${test_options[${test}]}"
        done
        echo "Checking decoder results"
        # Next, compare the tests against the reference
        for test in "${test_names[@]}"
        do
            testlog=trace_${data}_${test}.log
            reflog=trace_${data}_${ref}.log
            diff_count=`diff ${reflog} ${testlog} | wc -l`
            printf "Machine: %s; Data: %s; Test: %28s; Diff Count: %s\n" ${machine} ${data} ${test} ${diff_count}
            diff ${reflog} ${testlog}
        done
    done
    popd
done
