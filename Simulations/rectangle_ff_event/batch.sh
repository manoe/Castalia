#!/bin/sh -x

#PROTOS="efmrp shmrp hdmrp flooding"
#PROTOS="hdmrp"
PROTOS=shmrp

./gen_phy_proc.sh rectangular

rm Castalia-Trace.txt

for i in $PROTOS
do
    SEED=1678184087
    ./gen_routing.sh $i
    ./gen.sh omnetpp.ini qos_pdr=0.6 $SEED ${i}.yaml
    mv nrg.yaml ${i}_nrg.yaml
    mv pdr.yaml ${i}_pdr.yaml
    mv pkt.yaml ${i}_pkt.yaml
done

