#!/bin/sh -x

echo "config file:" $1
CFG=$1
echo "args: " $2
ARG=$2
echo "repeats:" $3
SEED=$3
echo "out file:" $4
OUT=$4

Castalia3 -i $CFG -c seed=$SEED,$ARG

yq -i e '. += { "loc_pdr": load("loc_pdr.yaml") }' pdr.yaml
yq -i '.runs += [ load("pdr.yaml")]' $OUT

