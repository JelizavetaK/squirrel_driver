#!/bin/sh
MYARCH=$(uname -m)
sudo apt-get install dpkg
sudo dpkg -i ft17driver_0.0-1_${MYARCH}.deb
