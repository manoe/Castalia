#!/bin/sh

case $1 in

  "none")
    echo "include phy_proc_none.ini" > physical_process.ini
    ;;

  "northeast")
    echo "include phy_proc_northeast.ini" > physical_process.ini
    ;;

  "center")
    echo "include phy_proc_center.ini" > physical_process.ini
    ;;

  "rectangular")
    echo "include phy_proc_rectangular.ini" > physical_process.ini
    ;;

  *)
    echo "unknown placement"
    ;;
esac

