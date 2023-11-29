#!/bin/sh

rm master.ini
rm no_depletion.ini
touch master.ini
touch no_depletion.ini

NODES=`python3 ./rand_master.py -n $1 -r $2 $3`

echo $NODES

for n in $NODES
do
    echo "SN.node[${n}].Application.isMaster = true" >> master.ini
    echo "SN.node[${n}].ResourceManager.isMaster = true" >> no_depletion.ini
done

echo "SN.node[0].Application.isMaster = true" >> master.ini
echo "SN.node[0].ResourceManager.isMaster = true" >> no_depletion.ini
    
