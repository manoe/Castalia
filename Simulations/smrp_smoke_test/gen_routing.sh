#!/bin/sh

case $1 in

  "shmrp")
    echo "include shmrp.ini" > routing.ini
    ;;

  "efmrp")
    echo "include efmrp.ini" > routing.ini
    ;;

  "aodv")
    echo "include aodv.ini" > routing.ini
    ;;

  "hdmrp")
    echo "include hdmrp.ini" > routing.ini
    ;;

  "flooding")
    echo "include flooding.ini" > routing.ini
    ;;

  "msr2mrp")
    echo "include msr2mrp.ini" > routing.ini
    ;;

  "smrp")
    echo "include smrp.ini" > routing.ini
    ;;

  *)
    echo "unknown protocol"
    ;;
esac

