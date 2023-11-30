#!/bin/sh -x

PROTO="shmrp"
ITER=10
SEED=`date +%s`
SEED_SET=`./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
EMR_EPSILON="0.0 0.05 0.1 0.15 0.2"

for e in ${EMR_EPSILON}
do
    yq --null-input '{"runs": []}' > shmrp_emr_${e}_pdr.yaml
done

./gen_phy_proc.sh rectangular
./gen_routing.sh ${PROTO}

for s in $SEED_SET
do
    for e in ${EMR_EPSILON}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,epsilon_emr=${e} ${s} shmrp_emr_${e}_pdr.yaml
    done
done

