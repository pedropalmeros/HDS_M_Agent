#! /bin/bash

. $FLAIR_ROOT/flair-dev/scripts/ubuntu_cgroup_hack.sh

if [ -f /proc/xenomai/version ];then
	EXEC=./FFormation07_17_11_2017_rt

else
	EXEC=./FFormation07_17_11_2017_nrt
fi

$EXEC -n x4_2 -a 127.0.0.1 -p 9000 -l ./ -x setup_x4_2.xml -t x4_simu2 -b 127.255.255.255:20010 -d 21002
