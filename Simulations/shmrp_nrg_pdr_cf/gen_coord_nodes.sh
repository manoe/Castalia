#!/bin/sh -x

rm nrg.ini
cp nrg_base.ini nrg.ini

NODES=`python3 ./failr.py -n ${1} -tt -f ${2}`

echo $NODES

for n in $NODES
do
    echo "SN.node[${n}].ResourceManager.initialEnergy = 20" >> nrg.ini
done
