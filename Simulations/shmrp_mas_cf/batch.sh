#!/bin/sh -x

PROTO=shmrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
MASTER_MU="0.0 0.1 0.2 0.4 0.5 0.6 0.8 1.0"
M_RATIO=20

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}

for n in ${MASTER_MU}
do
    yq --null-input '{"runs": []}' > ${PROTO}_mas_${n}_pdr.yaml
done

for s in ${SEED_SET}
do
    ./gen_master.sh 64 ${M_RATIO} ${s}
    for n in ${MASTER_MU}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,mu_master=${n} ${s} ${PROTO}_mas_${n}_pdr.yaml
    done
done

