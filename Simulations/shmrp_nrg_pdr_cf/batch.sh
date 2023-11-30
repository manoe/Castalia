#!/bin/sh -x

PROTO=shmrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
NRG_ETA="0.0 0.1 0.2 0.4 0.5 0.6 0.8 1.0"

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}
for n in ${NRG_ETA}
do
    yq --null-input '{"runs": []}' > ${PROTO}_nrg_${n}_pdr.yaml
done

for s in ${SEED_SET}
do
    for n in ${NRG_ETA}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,eta_nrg=${n} ${s} ${PROTO}_nrg_${n}_pdr.yaml
    done
done

