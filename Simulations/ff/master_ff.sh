#!/bin/sh

for i in ff_test_01_hdmrp_rreq.ini ff_test_01_flooding.ini
do
    for j in p10 p20 p30 diag
    do
        ./shuf.py > masternodes.ini
        ../../bin/Castalia -i $i -r 2 -c $j -o $i.$j.txt
    done
done
