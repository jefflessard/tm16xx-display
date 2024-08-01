#!/bin/bash

modprobe -a ledtrig_timer ledtrig_netdev tm16xx
echo "softdep tm16xx pre: ledtrig_timer ledtrig_netdev" > /etc/modprobe.d/tm16xx.conf
cp display-service /usr/sbin/
cp display.service /lib/systemd/system/

systemctl daemon-reload
systemctl enable display
systemctl start display
