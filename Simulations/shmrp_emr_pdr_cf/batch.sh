#!/bin/sh -x

PROTO="shmrp"
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
EMR_EPSILON=`seq 0.0 0.1 0.3`
PDR_PI=`seq 0.0 0.1 0.3`

for e in ${EMR_EPSILON}
do
    for p in ${PDR_PI}
    do
        yq --null-input '{"runs": []}' > shmrp_emr_${e}_pdr_${p}_pdr.yaml
        yq --null-input '{"runs": []}' > shmrp_emr_${e}_pdr_${p}_nrg.yaml
    done
done

./gen_phy_proc.sh rectangular
./gen_routing.sh ${PROTO}

for s in $SEED_SET
do
    for e in ${EMR_EPSILON}
    do
        for p in ${PDR_PI}
        do
            ./gen.sh omnetpp.ini qos_pdr=0.4,epsilon_emr=${e},pi_pdr=${p} ${s} shmrp_emr_${e}_pdr_${p}_pdr.yaml
            yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_emr_${e}_pdr_${p}_nrg.yaml
        done
    done
done

