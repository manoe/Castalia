#!/bin/sh -x

REP=1

#SEQ=`seq 11 5 31`

SEQ="0.8"

for j in $SEQ
do
yq --null-input '{"runs": []}' > shmrp_meas_qos_pdr_`echo $j|sed 's/\./_/g'`.yaml
done

# sed 's/\./_/g'


for i in $(seq 1 $REP)
do
     SEED=1678184087
     for j in $SEQ
	do
		./gen.sh omnetpp.ini qos_pdr=$j $SEED shmrp_meas_qos_pdr_`echo $j|sed 's/\./_/g'`.yaml
	done
done

tail Castalia-Trace.txt
