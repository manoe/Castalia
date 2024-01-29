#!/bin/sh -x

rm master.ini
touch master.ini

NODES=`python3 ./rand_master.py -n $1 -r $2 $3`

echo $NODES

for n in $NODES
do
    echo "SN.node[${n}].Application.isMaster = true" >> master.ini
done

echo "SN.node[0].Application.isMaster = true" >> master.ini
