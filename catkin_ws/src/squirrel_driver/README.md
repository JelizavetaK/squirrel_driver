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
- Set a manual IP in the 192.168.1.1/24 subnet, e.g. 192.168.1.100cd D with a subnet mask 255.255.255.0
- Install the FT17_driver as described above
- Check that the port name of Atduino board corresponds to the port name defined in main.cpp, line 23.

Execute:
  rosrun squirrel_sensing_node sensing [arduino_port_name] [ft17_port_name]
For example, rosrun squirrel_sensing_node sensing dev/tty0 eth0
  Two topics are published:
    /wrist  - that publishes [Fx, Fy, Fz, Mx, My, Mz]
    /fingertips - that publishes [Fz1, Mx1, My1, Fz2, Mx2, My2, Fz3, Mx3, My3, Dd1, Dp1, Dd2, Dp2, Dd3, Dp3], Dp stands for pad proximity sensor, and Dd stands for distal proximity sensor. Force is measured in N, moments in N/m, and distance in mm.

Proximity sensors display the value of -100 in case of no reading. The sensor does not measure beyond -20 mm. 

3. If you do not have FT sensor connected, then in file node.h comment line 10 (#define _FT17_AVAIL).
   
4. Information about sensors safety use and calibration (including setting to zero and saturation removal) can be found in squirrel_driver/squirrel_sensing_node/doc.
    
5. Make sure that you are using correct calibration file for your setup - squirrel_sensing_node/tactile_calibration.ini. Calibration files for each setup of the senors are located in squirrel_sensing_node/doc. 

