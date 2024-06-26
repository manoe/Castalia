#!/bin/sh -x

PROTO=shmrp
ITER=10
SEED=`date +%s`
SEED_SET=`python3 ./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt
NRG_ETA="0.0 0.05 0.1"

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}
touch nrg.ini

for n in ${NRG_ETA}
do
    yq --null-input '{"runs": []}' > ${PROTO}_nrg_${n}_pdr.yaml
    yq --null-input '{"runs": []}' > ${PROTO}_nrg_${n}_nrg.yaml
    yq --null-input '{"runs": []}' > ${PROTO}_nrg_test_pdr.yaml
done

for s in ${SEED_SET}
do
     rm nrg.ini
     cp nrg_base.ini nrg.ini
    ./gen.sh omnetpp.ini qos_pdr=0.4,eta_nrg=0.0,sim_time=800s ${s} ${PROTO}_nrg_test_pdr.yaml
    ./gen_rand_nodes.sh 6 ${s} "loc_pdr.yaml"
    for n in ${NRG_ETA}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.4,eta_nrg=${n} ${s} ${PROTO}_nrg_${n}_pdr.yaml
        yq -i '.runs += [ load("nrg.yaml")]' ${PROTO}_nrg_${n}_nrg.yaml
    done
done

