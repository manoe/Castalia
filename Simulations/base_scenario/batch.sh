#!/bin/sh -x

PROTOS="efmrp shmrp flooding hdmrp"

for i in $PROTOS
do
    SEED=1678184087
    ./gen_routing.sh $i
    ./gen.sh omnetpp.ini qos_pdr=0.8 $SEED ${i}.yaml
    mv nrg.yaml ${i}_nrg.yaml
done

