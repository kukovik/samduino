#!/bin/sh

lsmod | grep -q ftdi_sio && sudo rmmod ftdi_sio && NEED_MODPROBE=1

`dirname $0`/btld $*

[ "$NEED_MODPROBE" = "1" ] && sudo modprobe ftdi_sio

