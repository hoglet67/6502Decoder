#!/bin/bash

DECODE=../decode6502

MAXDIFFSIZE=500000000

common_options="--phi2="

declare -a external_master_files

TEST_FILE_URL="https://github.com/hoglet67/6502Decoder/releases/download/test_data"

declare -a external_master_test_files

external_master_test_files=(
    dormann_d6502.bin.gz
    dormann_d65c10.bin.gz
    clark_bcd_full.bin.gz
)

declare -a machine_names

machine_names=(
    master
    beeb
    elk
)

declare -A machine_options

machine_options[master]="--machine=master -c --vecrst=A9E364"
machine_options[beeb]="--machine=default --vecrst=A9D9CD"
machine_options[elk]="--machine=elk --vecrst=A9D8D2"

declare -a data_names

data_names=(
    reset
    dormann_d6502
    dormann_d65c10
    clark_bcd_full
)

declare -A data_options

data_options[reset]="-h -s"
data_options[dormann_d6502]="-h -s"
data_options[dormann_d65c10]="-h -s"
data_options[clark_bcd_full]="-h -s"

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
ref=${test_names[0]}

for file in "${external_master_test_files[@]}"
do
    echo "Downloading $file"
    wget -nv -N -P master ${TEST_FILE_URL}/${file}
done


for machine in "${machine_names[@]}"
do
    for data in "${data_names[@]}"
    do
        if [ -f ${machine}/${data}.bin.gz ]; then
            # First, generate all the data for the test cases
            echo "=============================================================================="
            echo "Running ${machine} ${data} tests"
            echo "=============================================================================="
            echo
            gunzip < ${machine}/${data}.bin.gz > ${machine}/${data}.tmp
            refmd5=""
            reflog=""
            for test in "${test_names[@]}"
            do
                log=${machine}/trace_${data}_${test}.log
                runcmd="${DECODE} ${common_options} ${data_options[${data}]} ${machine_options[${machine}]} ${test_options[${test}]} ${machine}/${data}.tmp > ${log}"
                echo "Test: ${test}"
                echo "  % ${runcmd}"
                eval $runcmd
                # If the file contains a RESET marker, prune any lines before this
                if grep -q RESET ${log}; then
                    sed -n '/RESET/,$p' < ${log} > tmp.log
                    mv tmp.log ${log}
                fi
                fail_count=`grep fail ${log} | wc -l`
                md5=`md5sum ${log} | cut -c1-8`
                size=$(stat -c%s "${log}")
                echo "  Trace MD5: ${md5}; Prediction fail count: ${fail_count}"
                # Log some context around each failure (limit to 100 failures)
                # Compare md5 of results with ref, rather than using diff, as diff can blow up
                if [ "${test}" == "${ref}" ]; then
                    refmd5=${md5}
                    reflog=${machine}/trace_${data}_${ref}.log
                    echo "  this is the reference trace"
                    if [ "${fail_count}" != "0" ]; then
                        echo
                        grep -10 -m 100 fail ${log}
                        echo
                    fi
                elif [ "${md5}" == "${refmd5}" ]; then
                    echo -e "  \e[32mPASS\e[97m: test trace matches reference trace"
                else
                    if [ "${fail_count}" != "0" ]; then
                        echo
                        grep -10 -m 100 fail ${log}
                        echo
                    fi
                    difcmd="diff ${reflog} ${log}"
                    if (( size < MAXDIFFSIZE )); then
                        diff_count=`${difcmd} | wc -l`
                        echo -e "  \e[31mFAIL\e[97m: test trace doesn't match reference trace; Diff Count: " ${diff_count}
                    else
                        echo -e "  \e[31mFAIL\e[97m: test trace doesn't match reference trace; file too large too diff"
                    fi
                    echo "  % ${difcmd}"
                fi
                echo
            done
        fi
    done
done
