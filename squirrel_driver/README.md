This squirrel_driver is composed of two parts, the ROS node (node.cpp/.h) and the hardware drivers (sensing_drivers.cpp/.h). 

1. First you need to install FT17 driver to your machine.
    
  To install the FT17 driver:

    Execute:
      cd ft17_driver
      ./install.sh

  This script will install the dynamic library libFT17_driver.so inside /usr/lib and all the related headers file in /usr/include. Then copy those filees to common lib and include them amnualy if they are not there.

2. To run the sensor: 
  Prerequisites
- Connect the ethernet cable from the PC to the FT17 sensor.
- Set a manual IP in the 192.168.1.1/24 subnet, e.g. 192.168.1.100 with a subnet mask 255.255.255.0
- Install the FT17_driver as described above

Execute:
  rosrun squirrel_sensing_node sensing
  Two topics are published:
    /wrist  - that publishes [Fx, Fy, Fz, Mx, My, Mz]
    /fingertips - that publishes [Fz1, Mx1, My1, Fz2, Mx2, My2, Fz3, Mx3, My3, Dp1, Dd1, Dp2, Dd2, Dp3, Dd3], Dp stands for pad proximity sensor, and Dd stands for distal proximity sensor.
    
    More information about sensors safety use and calibration can be found in squirrel_driver/squirrel_sensing_node/doc.
    

