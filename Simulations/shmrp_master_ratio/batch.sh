#!/bin/sh -x

PROTO=shmrp
M_RATIO="0 10 20 30 50 100"
MASTER="master no_depletion"
ITER=10

SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`

echo $SEED_SET > seed_set.txt

./gen_routing.sh $PROTO
./gen_phy_proc.sh none

for r in $M_RATIO
do
    for m in $MASTER
    do
        yq --null-input '{"runs": []}' > ${PROTO}_${r}_${m}_pkt.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_${r}_${m}_nrg.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_${r}_${m}_pdr.yaml
    done
done

for s in $SEED_SET
do
    for r in $M_RATIO
    do
    ./gen_master.sh 64 ${r} $s
        for m in $MASTER
        do
            ./gen.sh omnetpp.ini ${m} ${s} ${PROTO}_${r}_${m}_pdr.yaml
            yq -i '.runs += [ load("pkt.yaml")]' ${PROTO}_${r}_${m}_pkt.yaml
            yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_${r}_${m}_nrg.yaml
        done
    done
done

