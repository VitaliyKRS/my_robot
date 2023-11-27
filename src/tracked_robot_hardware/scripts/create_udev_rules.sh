#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ugv"
echo "ugv driver usb connection as /dev/ugv , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy ugv-driver.rules to  /etc/udev/rules.d/"
colcon_cd tracked_robot_hardware
sudo cp scripts/ugv-driver.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
