#!/bin/env bash

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

  "custom")
    if [[ ${#} -gt 1 ]] ; then
        export START_X=${2}
        export START_Y=${3}
    else
        export START_X=114
        export START_Y=114
    fi
    envsubst < phy_proc_custom.tmpl > physical_process.ini
  ;;

  *)
    echo "unknown placement"
  ;;
esac

