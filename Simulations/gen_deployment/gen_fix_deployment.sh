#!/bin/env bash

# $1 x number of nodes
# $2 y number of nodes
# $3 cell size
# $4 sim time

export CELL_SIZE=`echo $3`
export SIM_TIME=`echo $4`
export FIELD_X=`echo "${3} * (${1} - 1)"|bc `
export FIELD_Y=`echo "${3} * (${2} - 1)"|bc `
export NUM_NODES=`echo "${1} * ${2}"|bc `
export NUM_NODES_NS=`echo "${NUM_NODES}-1"|bc`
export NODES_X=$1
export NODES_Y=$2
export RANDOM_PREFIX="randomized_"
export RND_RANGE=0.1

#envsubst < deployment_fix.tmpl > deployment.ini

for y in $(seq 0 $((${NODES_Y} - 1))); do
    for x in $(seq 0 $((${NODES_X} - 1))); do
        NODE_NUM=$((${x} + ${y} * ${NODES_X}))
        echo SN.node[${NODE_NUM}].xCoor = $((${x} * ${CELL_SIZE} ))  
    done
done

