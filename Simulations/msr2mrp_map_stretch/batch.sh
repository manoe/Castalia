#!/bin/sh -x

PROTO=msr2mrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt

X_NODE=8
Y_NODE=8

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}

for i in `seq 1 0.1 2`
do
    ./gen_deployment.sh ${X_NODE} ${Y_NODE} `echo "${i} * 22.8"|bc`
    yq --null-input '{"runs": []}' > ${PROTO}_map_scale_${i}_pdr.yaml
    for s in ${SEED_SET}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square ${s} ${PROTO}_map_scale_${i}_pdr.yaml
    done
done
