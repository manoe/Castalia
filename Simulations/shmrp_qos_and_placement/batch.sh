#!/bin/sh -x

PROTO=shmrp
QOS="0.0 0.2 0.4 0.6 0.8"
PLACEMENT="center side corner"
ITER=10

SEED=`date +%s`
SEED_SET=`./rand.py -i $ITER $SEED`

./gen_routing.sh $PROTO

for q in $QOS
do
    for p in $PLACEMENT
    do
        yq --null-input '{"runs": []}' > ${PROTO}_${q}_${p}_pkt.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_${q}_${p}_nrg.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_${q}_${p}_pdr.yaml
    done
done

for s in $SEED_SET
do
    for q in $QOS
    do
        for p in $PLACEMENT
        do
            ./gen.sh omnetpp.ini qos_pdr=$q,$p $SEED ${PROTO}_${q}_${p}_pdr.yaml
            yq -i '.runs += [ load("pkt.yaml")]' ${PROTO}_${q}_${p}_nrg.yaml
            yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_${q}_${p}_pdr.yaml

        done
    done
done

