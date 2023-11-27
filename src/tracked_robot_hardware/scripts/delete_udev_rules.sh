#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  ugv"
echo "sudo rm   /etc/udev/rules.d/ugv-driver.rules"
sudo rm   /etc/udev/rules.d/ugv-driver.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
