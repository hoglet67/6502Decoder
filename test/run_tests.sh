#!/bin/bash

DECODE=../decode6502

common_options="--phi2= -h -s"

declare -a machine_names
machine_names=(master beeb)

declare -A machine_options
machine_options[master]="--machine=master -c --vecrst=A9E364"
machine_options[beeb]="--vecrst=A9D9CD"
machine_options[electon]="--vecrst=A9D8D2"

declare -a test_names
test_names=(
sync
sync_nornw
sync_norst
sync_nordy
sync_nornw_norst
sync_nornw_nordy
sync_norst_nordy
sync_nornw_norst_nordy
nosync
nosync_nornw
nosync_norst
nosync_nordy
nosync_nornw_norst
nosync_nornw_nordy
nosync_norst_nordy
nosync_nornw_norst_nordy
)

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

for machine in "${machine_names[@]}"
do
    for data in reset
    do
        # First, generate all the data for the test cases
        echo "=============================================================================="
        echo "Running ${machine} ${data} tests"
        echo "=============================================================================="
        echo
        for test in "${test_names[@]}"
        do
            log=${machine}/trace_${data}_${test}.log
            cmd="gunzip < ${machine}/${data}.bin.gz | ${DECODE} ${common_options} ${machine_options[${machine}]} ${test_options[${test}]} > ${log}"
            eval $cmd
            # If the file contains a RESET marker, prune any lines before this
            if grep -q RESET ${log}; then
                sed -n '/RESET/,$p' < ${log} > tmp.log
                mv tmp.log ${log}
            fi

            fail_count=`grep fail ${log} | wc -l`
            md5=`md5sum ${log} | cut -c1-8`
            printf "Machine: %s; Data: %s; Test: %28s; MD5: %s; Fail Count: %s\n" ${machine} ${data} ${test} ${md5} ${fail_count}
            echo "  % ${cmd}"
            echo
        done
        echo "=============================================================================="
        echo "Checking ${machine} ${data} results"
        echo "=============================================================================="
        echo
        # Next, compare the tests against the reference
        for test in "${test_names[@]}"
        do
            testlog=${machine}/trace_${data}_${test}.log
            reflog=${machine}/trace_${data}_${ref}.log
            cmd="diff ${reflog} ${testlog}"
            diff_count=`${cmd} | wc -l`
            printf "Machine: %s; Data: %s; Test: %28s; Diff Count: %s\n" ${machine} ${data} ${test} ${diff_count}
            echo "  % ${cmd}"
            echo
        done
    done
done
