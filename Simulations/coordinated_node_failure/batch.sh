#!/bin/sh -x

PROTO="shmrp efmrp hdmrp"
FAIL=3
NUM=`expr $FAIL + 1`

SEED=1678184087

for p in $PROTO
do
    rm loc_pdr.yaml
    rm base_loc_pdr.yaml
    rm fail.ini
    touch fail.ini

    for i in `seq $NUM`
    do
        if [ -f Castalia-Trace.txt ]
        then
            rm Castalia-Trace.txt
        fi
        ./gen_routing.sh $p
        ./gen_sim.sh `expr 1800 \* ${i}`
        ./gen.sh omnetpp.ini qos_pdr=0.8 $SEED omnetpp.yaml
        if [ $FAIL -ge $i ]
        then
            if [ -f base_loc_pdr.yaml ]
            then
                ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml -b base_loc_pdr.yaml` `expr 1800 \* ${i}`
            else
                ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml` `expr 1800 \* ${i}`
            fi
        mv loc_pdr.yaml base_loc_pdr.yaml
        fi
    done
    
    mv nrg.yaml ${p}_nrg.yaml
done
