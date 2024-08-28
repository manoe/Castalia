#!/bin/sh -x

PROTO=msr2mrp
ITER=1
#SEED=`date +%s`
#SEED_SET=`python3 ./rand.py -i $ITER $SEED`
SEED_SET=597428753
#echo $SEED_SET > seed_set.txt
#SCALE=`seq 1 0.1 2`
SCALE=1

X_NODE=8
Y_NODE=8

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}

for i in $SCALE
do
    ./gen_deployment.sh ${X_NODE} ${Y_NODE} `echo "${i} * 22.8"|bc` 4h
    yq --null-input '{"runs": []}' > ${PROTO}_map_scale_${i}_pdr.yaml
    for s in ${SEED_SET}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square ${s} ${PROTO}_map_scale_${i}_pdr.yaml
    done
done
