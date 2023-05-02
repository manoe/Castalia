#!/bin/sh

for i in  ff_test_01_hdmrp_rreq.ini ff_test_01_hdmrp_norreq.ini #ff_test_01_flooding.ini  ff_test_01_hdmrp_fix.ini
do
#    ../../bin/Castalia -i $i -r 5 -c left -o $i.left.txt
    ../../bin/Castalia -i $i -r 2  -o $i.corner.txt
    ../../bin/Castalia -i $i -r 2 -c left -o $i.left.txt
    ../../bin/Castalia -i $i -r 2 -c center -o $i.center.txt
done
