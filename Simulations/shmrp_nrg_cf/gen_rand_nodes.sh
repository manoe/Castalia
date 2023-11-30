#!/bin/sh -x

rm nrg.ini
cp nrg_base.ini nrg.ini

NODES=`python3 ./rand_sel.py -s ${2} -n ${1} -f ${3}`

echo $NODES

for n in $NODES
do
    echo "SN.node[${n}].ResourceManager.initialEnergy = 20" >> nrg.ini
done
