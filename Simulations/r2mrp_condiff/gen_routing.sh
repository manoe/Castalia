#!/bin/env sh

case $1 in

  "r2mrp")
    echo "include r2mrp.ini" > routing.ini
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

  *)
    echo "unknown protocol"
    ;;
esac

