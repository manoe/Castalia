#!/bin/sh -x

PROTOS="efmrp shmrp hdmrp flooding"
ITER=10
SEED=`date +%s`
SEED_SET=`./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt

for p in $PROTOS
do
    yq --null-input '{"runs": []}' > ${p}_pdr.yaml
done

./gen_phy_proc.sh rectangular

for s in $SEED_SET
do
    for p in $PROTOS
    do
        ./gen_routing.sh $p
        ./gen.sh omnetpp.ini qos_pdr=0.6 ${s} ${p}_pdr.yaml
    done
done

