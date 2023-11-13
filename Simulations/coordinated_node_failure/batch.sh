#!/bin/sh -x

#PROTOS="efmrp shmrp hdmrp flooding"
PROTO="shmrp"
FAIL=0
NUM=`expr $FAIL + 1`
rm loc_pdr.yaml
rm base_loc_pdr.yaml
rm fail.ini
touch fail.ini

SEED=1678184087


for i in `seq $NUM`
do
    if [ -f Castalia-Trace.txt ]
    then
        rm Castalia-Trace.txt
    fi
    ./gen_routing.sh $PROTO
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

mv nrg.yaml ${PROTO}_nrg.yaml
