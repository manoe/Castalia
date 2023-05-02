#!/bin/sh

PREFIX=node20
COUNT=3

for i in  ff_test_01_hdmrp_rreq.ini ff_test_01_flooding.ini 
do
    for j in ff_ne ff_center ff_sw ff_se
    do
        for k in p10 diag nomaster
        do
            for l in event120
            do
            ../../bin/Castalia -i $i -r 2  -c $j,$k,$l       -o $i.$j.$k.$l.$PREFIX.corner.txt
            ../../bin/Castalia -i $i -r 2 -c $j,left,$k,$l   -o $i.$j.$k.$l.$PREFIX.left.txt
            ../../bin/Castalia -i $i -r 2 -c $j,center,$k,$l -o $i.$j.$k.$l.$PREFIX.center.txt

            echo "scale = 2 ; $COUNT / 144 * 100" |bc
            COUNT=`expr $COUNT + 3`
            done
        done
    done
done
