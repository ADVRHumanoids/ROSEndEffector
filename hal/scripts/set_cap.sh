#! /bin/sh
sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice=eip $1
sudo getcap $1
