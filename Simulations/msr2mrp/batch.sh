#!/bin/sh -x

PROTO=msr2mrp
ITER=1
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
SEED_SET=597428753
#echo $SEED_SET > seed_set.txt

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}
yq --null-input '{"runs": []}' > ${PROTO}_pdr.yaml

for s in ${SEED_SET}
do
    ./gen.sh omnetpp.ini qos_pdr=0.6,sim_time=20min ${s} ${PROTO}_pdr.yaml
done

