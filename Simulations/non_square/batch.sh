#!/bin/sh -x

PROTO=shmrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
MAS_MU=`seq 0.0 0.1 0.3`
PDR_PI=`seq 0.0 0.1 0.3`
NODE_NUM=64
M_RATIO=30

#
# POSITION????
#
./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}
touch master.ini

for n in ${MAS_MU}
do
   for p in ${PDR_PI}
    do
        yq --null-input '{"runs": []}' > ${PROTO}_mas_${n}_pdr_${p}_pdr.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_mas_${n}_pdr_${p}_nrg.yaml
        yq --null-input '{"runs": []}' > ${PROTO}_mas_test_pdr.yaml
    done
done

for s in ${SEED_SET}
do
     rm master.ini
     touch master.ini
    ./gen.sh omnetpp.ini qos_pdr=0.4,mu_mas=0.0,pi_pdr=0.0,sim_time=800s ${s} ${PROTO}_mas_test_pdr.yaml
    ./gen_master.sh ${NODE_NUM} ${M_RATIO} ${s}
    yq --null-input '{"runs": []}' > ${PROTO}_nrg_test_pdr.yaml

    for n in ${MAS_MU}
    do
        for p in ${PDR_PI}
        do
            ./gen.sh omnetpp.ini qos_pdr=0.4,mu_mas=${n},pi_pdr=${p} ${s} ${PROTO}_mas_${n}_pdr_${p}_pdr.yaml
            yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_mas_${n}_pdr_${p}_nrg.yaml
        done
    done
done

