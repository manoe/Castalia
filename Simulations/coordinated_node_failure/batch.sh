#!/bin/sh -x

#PROTO="shmrp efmrp hdmrp"
PROTO="shmrp"
FAIL=1
ITER=1
NUM=`expr $FAIL + 1`

SEED=`date +%s`
SEED_SET=`./rand.py -i $ITER $SEED`

execute_iter() {
    local INVAL=$1
    local OFFSET=$2
    for i in `seq $INVAL`
    do
        if [ -f Castalia-Trace.txt ]
        then
            rm Castalia-Trace.txt
        fi
        ./gen_routing.sh $p
        ./gen_sim.sh `expr 1800 \* ${i} \+ $OFFSET`
        ./gen.sh omnetpp.ini qos_pdr=0.8 $r ${p}_pdr.yaml
        if [ $FAIL -ge $i ]
        then
            if [ -f base_loc_pdr.yaml ]
            then
                ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml -b base_loc_pdr.yaml` `expr 1800 \* ${i} \+ $OFFSET`
            else
                ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml` `expr 1800 \* ${i} \+ $OFFSET`
            fi
        mv loc_pdr.yaml base_loc_pdr.yaml
        fi
    done
}

for p in $PROTO
do
    yq --null-input '{"runs": []}' > ${p}_pkt.yaml
    yq --null-input '{"runs": []}' > ${p}_nrg.yaml
    yq --null-input '{"runs": []}' > ${p}_pdr.yaml
    for r in $SEED_SET
    do
        rm loc_pdr.yaml
        rm base_loc_pdr.yaml
        rm fail.ini
        touch fail.ini

        execute_iter $NUM "0"
        execute_iter $NUM "7200"
        

        mv fail.ini ${p}_${r}_fail.ini
        yq -i '.runs += [ load("pkt.yaml")]' ${p}_pkt.yaml
        yq -i '.runs += [ load("nrg.yaml")]' ${p}_nrg.yaml
    done
done
