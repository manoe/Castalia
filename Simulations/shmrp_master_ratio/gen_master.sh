#!/bin/sh

rm master.ini
rm no_depletion.ini
touch master.ini
touch no_depletion.ini

SEED=`date +%s`
NODES=`python3 ./rand_master.py -n $1 -r $2 $SEED`

echo $NODES

for n in $NODES
do
    echo "SN.node[${n}].Application.isMaster = true" >> master.ini
    echo "SN.node[${n}].ResourceManager.isMaster = true" >> no_depletion.ini
done
    
