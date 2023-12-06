#!/bin/sh -x

PROTO=shmrp
ITER=1
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
TIMES="3599"

./gen_phy_proc.sh custom
./gen_routing.sh ${PROTO}

for t in ${TIMES}
do
    yq --null-input '{"runs": []}' > ${PROTO}_ff_time_${t}_pdr.yaml
    yq --null-input '{"runs": []}' > ${PROTO}_ff_time_${t}_nrg.yaml
done

for s in ${SEED_SET}
do
    for t in ${TIMES}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,sim_time=${t}s ${s} ${PROTO}_ff_time_${t}_pdr.yaml
        yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_ff_time_${t}_nrg.yaml
    done
done

