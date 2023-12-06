#!/bin/sh

export SIM_TIME=`echo "60 * ${1}^2"|bc`
export FIELD_X=`echo "22.8 * (${1} - 1)"|bc `
export FIELD_Y=`echo "22.8 * (${1} - 1)"|bc `
export NUM_NODES=`echo "${1}^2"|bc `
export NUM_NODES_NS=`echo "${NUM_NODES}-1"|bc`
export NODES_X=$1
export NODES_Y=$1

envsubst < deployment.tmpl > deployment.ini
