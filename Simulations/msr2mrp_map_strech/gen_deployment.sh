#!/bin/sh

# $1 x $2 y $3 cell size

export SIM_TIME=`echo 3600`
export FIELD_X=`echo "${3} * (${1} - 1)"|bc `
export FIELD_Y=`echo "${3} * (${2} - 1)"|bc `
export NUM_NODES=`echo "${1} * ${2}"|bc `
export NUM_NODES_NS=`echo "${NUM_NODES}-1"|bc`
export NODES_X=$1
export NODES_Y=$2
export RANDOM="randomized_"
export RND_RANGE=0.1

envsubst < deployment.tmpl > deployment.ini
