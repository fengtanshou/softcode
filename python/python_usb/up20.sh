#########################################################################
# File Name: i2c.sh
# Author: Alex
# mail: fangyuan.liu@nio.com
# Created Time: Wed 25 Oct 2017 07:48:43 PM CST
#########################################################################
#!/bin/bash

interface=`ifconfig | grep enp0s20 | awk '{print $1}'`
echo $interface
sudo ifconfig $interface 192.168.1.10 up

ifconfig $interface


'''
interface=`ifconfig | grep enp0s31f6 | awk '{print $1}'`
echo $interface
sudo ifconfig $interface 172.20.1.1  up
ifconfig $interface
'''
