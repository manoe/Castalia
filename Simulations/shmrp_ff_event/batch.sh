#!/bin/sh -x

PROTO=shmrp
ITER=1

if [ "$1" ]
then
    SEED_SET=$1
else
    SEED=`date +%s`
    SEED_SET=`python3 ./rand.py -i $ITER $SEED`
    echo $SEED_SET > seed_set.txt
fi

TIMES="1799 5000"

./gen_phy_proc.sh custom
./gen_routing.sh ${PROTO}

for t in ${TIMES}
do
    yq --null-input '{"runs": []}' > ${PROTO}_ff_time_${t}_pdr.yaml
    yq --null-input '{"runs": []}' > ${PROTO}_ff_time_${t}_nrg.yaml
    yq --null-input '{"runs": []}' > ${PROTO}_ff_time_${t}_pkt.yaml
done

for s in ${SEED_SET}
do
    for t in ${TIMES}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,sim_time=${t}s,side ${s} ${PROTO}_ff_time_${t}_pdr.yaml
        yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_ff_time_${t}_nrg.yaml
        yq -i '.runs += [ load("pkt.yaml")]' ${PROTO}_ff_time_${t}_pkt.yaml
    done
done

