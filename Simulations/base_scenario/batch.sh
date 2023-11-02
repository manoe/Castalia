#!/bin/sh -x

PROTOS="efmrp shmrp flooding hdmrp"

for i in $PROTOS
do
    yq --null-input '{"runs": []}' > ${i}.yaml
    yq --null-input '{"runs": []}' > ${i}_pkt.yaml
done

for i in $PROTOS
do
    SEED=1678184087
    ./gen_routing.sh $i
    ./gen.sh omnetpp.ini qos_pdr=0.8 $SEED ${i}.yaml
    yq -i '.runs += [ load("pkt.yaml")]' ${i}_pkt.yaml
done

