#!/bin/sh -x

PROTO="shmrp hdmrp"
SEED=`date +%s`
#SEED_SET=`python3 ./rand.py -i $ITER $SEED`
SEED_SET=$SEED

for p in ${PROTO}
do
    yq --null-input '{"runs": []}' > ${p}_pdr.yaml
    yq --null-input '{"runs": []}' > ${p}_nrg.yaml
done

./gen_phy_proc.sh none

for s in $SEED_SET
do
    for p in ${PROTO}
    do
        ./gen_routing.sh ${p}
        ./gen.sh omnetpp.ini qos_pdr=0.6 ${s} ${p}_pdr.yaml
        yq -i '.runs += [ load("nrg.yaml")]' ${p}_nrg.yaml
    done
done
