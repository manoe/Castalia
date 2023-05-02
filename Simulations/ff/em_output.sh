#!/bin/sh

PREFIX=node20
COUNT=3

#for i in  ff_test_01_hdmrp_rreq.ini ff_test_01_flooding.ini 
#do
    for j in ff_ne ff_center ff_sw ff_se
    do
        for k in p10 diag nomaster
        do
            for l in event120
            do
                for m in event consum orig 'fail --sum'
                do
            ../../bin/CastaliaResults ff_test_01_flooding.ini.$j.$k.$l.$PREFIX.center.txt -s $m
            ../../bin/CastaliaResults ff_test_01_flooding.ini.$j.$k.$l.$PREFIX.corner.txt -s $m
            ../../bin/CastaliaResults ff_test_01_flooding.ini.$j.$k.$l.$PREFIX.left.txt   -s $m
        done
            done
        done
    done
#done
