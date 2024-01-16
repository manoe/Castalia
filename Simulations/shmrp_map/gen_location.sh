LOC="0 22.857142857142858 45.714285714285715 68.571428571428569 91.428571428571431 114.28571428571429 137.14285714285714 160"
NODE_NUM=64
k=0
MASTER_NODE="0 9 18 27 36 45 54 63 44 52 60"

for i in $LOC
do
    for j in $LOC
    do
        echo SN.node[${k}].MobilityManager.positionOverride = true
        echo SN.node[${k}].xCoor = ${j}
        echo SN.node[${k}].yCoor = ${i}
        for m in $MASTER_NODE
        do
            if [ ${k} = ${m} ];
            then
                echo SN.node[${k}].Application.isMaster = true
            fi
        done
        k=`expr $k + 1`
    done
done
