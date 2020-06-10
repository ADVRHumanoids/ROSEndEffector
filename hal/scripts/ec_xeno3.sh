#! /bin/sh
### BEGIN INIT INFO
# Provides:          rtnet
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: rtnet
# Description:       rtnet
### END INIT INFO

RTNET_BIN_DIR=/usr/xenomai/sbin
export PATH=$PATH:$RTNET_BIN_DIR

ETH_IFACE=enp3s0

ETH_DRV=e1000e
RT_ETH_DRV=rt_$ETH_DRV

# PCI addresses of RT-NICs to claim (format: 0000:00:00.0)
#   If both Linux and RTnet drivers for the same hardware are loaded, this
#   list instructs the start script to rebind the given PCI devices, detaching
#   from their Linux driver, attaching it to the RT driver above. Example:
#   REBIND_RT_NICS="0000:00:19.0 0000:01:1d.1"
#REBIND_RT_NICS=$(ethtool -i $ETH_IFACE | grep bus-info | cut -d ' ' -f 2)
REBIND_RT_NICS="0000:03:00.0"
#REBIND_RT_NICS=""

. /lib/lsb/init-functions

case "$1" in
  start)
    log_action_msg "Configure rtnet" 

    chgrp xenomai /dev/rtp*
    chmod g+rw /dev/rtp*

    #/bin/echo "ifdown" $ETH_IFACE 
    #ifdown $ETH_IFACE 
    #rmmod $ETH_DRV
    /bin/echo "probing rtnet modules" 
    modprobe $RT_ETH_DRV 
    modprobe rtpacket

    echo "Rebind pci " $REBIND_RT_NICS
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$ETH_DRV/unbind
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$RT_ETH_DRV/bind
    
    /bin/echo "ifup" rteth0 
    rtifconfig rteth0 up
    log_end_msg $?
  ;;

  stop)
    log_action_msg "DeConfigure rtnet"
    rtifconfig rteth0 down 2> /dev/null
    rmmod rtpacket
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$RT_ETH_DRV/unbind
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$ETH_DRV/bind
    /bin/echo "ifup" $ETH_IFACE
    ifup $ETH_IFACE
    log_end_msg $?
  ;;
  restart|force-reload)
    $0 stop
    sleep 1
    $0 start
  ;;
  status)
    lsmod | grep rt
  ;;
  *)
    echo "Usage: $0 {start|stop|status}"
    exit 1
  ;;
esac

exit
