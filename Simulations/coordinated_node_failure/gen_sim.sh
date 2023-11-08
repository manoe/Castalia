#!/bin/sh

export SIM_TIME=${1}

envsubst < sim.tmpl > sim.ini
