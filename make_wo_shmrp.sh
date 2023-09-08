#!/bin/sh
#
make makefiles EXCEPT="--except node/communication/routing/shmrp" CFLAGS="-Wall -pedantic -Wextra"
