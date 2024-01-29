#!/bin/sh -x

PROTO=shmrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
X_NODE=4
Y_NODE=16
#
# POSITION????
#
./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}
./gen_deployment.sh ${X_NODE} ${Y_NODE}

PREFIX="${PROTO}_${X_NODE}x${Y_NODE}"

yq --null-input '{"runs": []}' > ${PREFIX}_pdr.yaml
yq --null-input '{"runs": []}' > ${PREFIX}_nrg.yaml

for s in ${SEED_SET}
do
    ./gen.sh omnetpp.ini non_square ${s} ${PREFIX}_pdr.yaml
    yq -i '.runs += [ load("nrg.yaml")]' ${PREFIX}_nrg.yaml
done
