#!/bin/sh -x

PROTOS="efmrp shmrp hdmrp flooding"

SEED=`date +%s`
SEED_SET=`./rand.py -i $ITER $SEED`
echo $SEED_SET > seed_set.txt

./gen_phy_proc.sh rectangular

for s in $SEED_SET
do
    for i in $PROTOS
    do
    ./gen_routing.sh $i
    ./gen.sh omnetpp.ini qos_pdr=0.6 $SEED ${i}.yaml
done

