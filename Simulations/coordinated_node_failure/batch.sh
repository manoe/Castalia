#!/bin/sh -x

#PROTOS="efmrp shmrp hdmrp flooding"
PROTO="efmrp"
NUM=3

rm loc_pdr.yaml
rm base_loc_pdr.yaml
rm fail.ini
touch fail.ini

SEED=1678184087

for i in `seq $NUM`
do
    ./gen_routing.sh $PROTO
    ./gen_sim.sh `expr 1800 \* ${i}`
    ./gen.sh omnetpp.ini qos_pdr=0.8 $SEED omnetpp.yaml
    if [ -f base_loc_pdr.yaml ]
    then
        ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml -b base_loc_pdr.yaml` `expr 1800 \* ${i}`
    else
        ./add_fail.sh `~/crunchr/failr/main.py -f loc_pdr.yaml` `expr 1800 \* ${i}`
    fi
    mv loc_pdr.yaml base_loc_pdr.yaml

done
