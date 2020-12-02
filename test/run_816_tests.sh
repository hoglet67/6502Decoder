#!/bin/bash

if [[ `uname` = Darwin ]]; then
  STATARGS=-f%z
else
  STATARGS=-c%s
fi


# Parse the command line options
POSITIONAL=()
REF=0
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -r|--ref|--reference)
            REF=1
            shift # past argument
            ;;
        *)    # unknown option
            POSITIONAL+=("$1") # save it in an array for later
            shift # past argument
            ;;
    esac
done

DECODE=../decode6502

common_options="-d1 -8 -a -h -i -y -s --sp=01FD --phi2= --rdy= --rst="

declare -A test_options

# Tests to run in emulation mode
test_options[romcopy_hipoke_17]="--emul=1"
test_options[reset]="--emul=1"
test_options[hog816_emu]="--emul=1"

# Tests to run in native mode
test_options[hog816_interrupt]="--emul=0"
test_options[hog816_native]="--emul=0"

for data in `find 816* -name '*.data' | sort`
do


    name=`basename ${data}`
    name=${name%.data}

    echo $name;

    log=${data%.data}.log
    ref=${data%.data}.ref

    # If the --ref option is given, then save the current log file as a reference
    if [ "${REF}" == "1" ]; then
        if [ -f ${log} ]; then
            mv ${log} ${ref}
        fi
    fi

    echo "${DECODE} ${common_options} ${test_options[${name}]} ${data} > ${log}"
    ${DECODE}       ${common_options} ${test_options[${name}]} ${data} > ${log}

    fail_count=`grep fail ${log} | wc -l`
    md5=`md5sum ${log} | cut -c1-8`
    size=$(stat ${STATARGS} "${log}")


    if [ -f ${ref} ]; then
        diff_count=`diff ${ref} ${log} | wc -l`
    else
        diff_count="-1"
    fi

    echo "  Trace MD5: ${md5}; Prediction fail count: ${fail_count}; Reference diff count: ${diff_count}"

    diff ${ref} ${log}

    echo


#        if [ "${fail_count}" != "0" ]; then
#            echo
#            grep -10 -m 100 fail ${log}
#            echo
#        fi

done
