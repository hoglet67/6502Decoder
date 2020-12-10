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

common_options="-d2 -8 -a -h -i -y -s --sp=01E0 --phi2= --rdy= --rst="

declare -A test_options

# Tests to run in emulation mode
              test_options[brk]="--emul=1 --pb=00 --db=00 --dp=0000"
          test_options[optest1]="--emul=1 --pb=00 --db=00 --dp=0000"
          test_options[optest2]="--emul=1 --pb=00 --db=00 --dp=0000"
          test_options[optest3]="--emul=1 --pb=00 --db=00 --dp=0000"
test_options[romcopy_hipoke_17]="--emul=1 --pb=00 --db=00 --dp=0000"
            test_options[reset]="--emul=1 --pb=00 --db=00 --dp=0000"
             test_options[test]="--emul=1 --pb=00 --db=00 --dp=0000"
       test_options[hog816_emu]="--emul=1 --pb=00 --db=00 --dp=0000"
    test_options[dormann_d6502]="--emul=1 --pb=00 --db=00 --dp=0000 --sp= --quiet"


# Tests to run in native mode
 test_options[hog816_interrupt]="--emul=0 --pb=01 --db=01 --dp=1900 --sp=01FD"
    test_options[hog816_native]="--emul=0 --pb=01 --db=01 --dp=1900"

for data in `find 816* -name '*.data' | sort`
do


    name=`basename ${data}`
    name=${name%.data}

    echo $name;

    log=${data%.data}.log
    ref=${data%.data}.ref
    dif=${data%.data}.dif

    echo "${DECODE} ${common_options} ${test_options[${name}]} ${data} > ${log}"
    ${DECODE}       ${common_options} ${test_options[${name}]} ${data} > ${log}

    fail_count=`grep fail ${log} | wc -l`
    md5=`md5sum ${log} | cut -c1-8`
    size=$(stat ${STATARGS} "${log}")


    diff_count="-1"

    # If the --ref option is given, then save the current log file as a reference
    if [ "${REF}" == "1" ]; then
        mv ${log} ${ref}
    else
        if [ -f ${ref} ]; then
            diff ${ref} ${log} | head -10000 > ${dif}
            diff_count=`wc -l <${dif}`
        fi
    fi

    echo "  Trace MD5: ${md5}; Prediction fail count: ${fail_count}; Reference diff count: ${diff_count}"


    echo


#        if [ "${fail_count}" != "0" ]; then
#            echo
#            grep -10 -m 100 fail ${log}
#            echo
#        fi

done
