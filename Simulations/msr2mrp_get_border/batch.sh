#!/bin/bash -x

PROTO=msr2mrp
ITER=1
#SEED=`date +%s`
#SEED_SET=`python3 ./rand.py -i $ITER $SEED`
SEED_SET=597428753
#echo $SEED_SET > seed_set.txt
#SCALE=`seq 1 0.1 2`
SCALE=1

X_NODE=8
Y_NODE=8

./gen_phy_proc.sh none
./gen_routing.sh ${PROTO}

for i in $SCALE
do
    ./gen_deployment.sh ${X_NODE} ${Y_NODE} `echo "${i} * 22.8"|bc`
    yq --null-input '{"runs": []}' > ${PROTO}_map_scale_${i}_pdr.yaml
    for s in ${SEED_SET}
    do
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square,short_sim,fivekall ${s} ${PROTO}_map_scale_${i}_pdr.yaml
#        borders=(`yq '.[] | select(.engines[].role == "border")| .node' loc_pdr.yaml`)
#        b_len=${#borders[@]}
#        echo length: ${b_len}
#        b_ind_len=`echo "${b_len} - 1"|bc`
#        e_b=`shuf -i 0-${b_ind_len} -n 1 --random-source=/dev/urandom`
        rm border.ini
        for n in $(seq 0 63)
        do
            echo "SN.node[${n}].ResourceManager.initialEnergy = 5000" >> border.ini

        done
        e_b=`~/crunchr/pick_border/main.py loc_pdr.yaml`
#        echo "SN.node[${borders[${e_b}]}].ResourceManager.initialEnergy = 2500" >> border.ini
        echo "SN.node[${e_b}].ResourceManager.initialEnergy = 2500" >> border.ini
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square,long_sim,e_border,topsis_w ${s} ${PROTO}_map_scale_${i}_pdr.yaml
        cp nrg.yaml nrg_topsis_w.yaml
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square,long_sim,e_border ${s} ${PROTO}_map_scale_${i}_pdr.yaml
        cp nrg.yaml nrg_topsis.yaml
        ./gen.sh omnetpp.ini qos_pdr=0.6,non_square,long_sim,e_border,rnd_routing ${s} ${PROTO}_map_scale_${i}_pdr.yaml
        cp nrg.yaml nrg_rnd.yaml


    done
done
