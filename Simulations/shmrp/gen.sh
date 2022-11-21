#!/bin/sh -x


echo "config file:" $1
CFG=$1
echo "args: " $2
ARG=$2
echo "repeats:" $3
REP=$3
echo "out file:" $4
OUT=$4

yq --null-input '{"runs": []}' > $4


for i in $(seq 1 $REP)
do
    SEED=`date +%s`
    Castalia -i $CFG -c seed=$SEED
    yq -i '.runs += [ load("pdr.yaml")]' out.yaml
done


