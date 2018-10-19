# Indoor Mapper
This project details the design of an indoor mapper project. This is an ongoing project, and will add capabilities over time


## Requirements 

- mobile robot for indoor environment
- indoor environment - carpeted floor, wood, or other plan surface. furniture 
- 2D navigation
- autonomous navigation in pre generated mapped environments
- map generation capability 
- max speed of 1 m/sec
- rechargeable/non-rechargeable battery
- remote controlling capability through Bluetooth/Wifi
- data streaming capability
- range finding accuracy +/5 cm
- in-place 360 degree rotation
- enough space to add jetson TK1/TX1, raspberry Pi, 32-bit uC
- sleep/wake-up capability to conserve battey life


- Future capabilities: temperature sensing, voice recognition, facial recognition

## BOM

|Part Name|Qty|Unit Price ($)|Total Price ($)|
|---------|---|----------|-----------|
|Waffle Board|6|10.00|40.00|
|[Dynamixel XL430-W250-T](http://www.robotis.us/x-series/?price_min=0&price_max=82&sort=featured)|2|49.50|99.00|
|[RPLIDAR-A1](https://www.seeedstudio.com/RPLiDAR-A1M8-360-Degree-Laser-Scanner-Kit-12M-Range-p-3072.html)|1|99.00|99.00|
|LiPo|1|50.00|50.00|
|Caster|1|5.00|1|
|PlateSupport M3x35mm|4|||
|PlateSupport M3x45mm|4|||
|Tyre|2|||
|Sprocket|2||


## Power Budget

|Component|Qty|Avg Idle I (mA)|Avg Max I (mA)|Peak I (mA)|Voltage (V)|V<sub>ripple</sub>(V/ %)|
|---------|---|---------------|----------------------|-----------|-----------|------------------------|
|R-Pi3    |1  |               |                      |           |           |                        |
|R-Pi3 + CAM|1|               |                      |           |           |                        |
|TIVA Launchpad|1|            |                      |           |           |                        |
|Dynamixel XL430-W250-T|2     |                      |           |           |                        |
|LiDAR    |1   |              |                      |           |           |                        |
|IR Sensor|5   |              |                      |           |           |                        |
|Nvidia TX1|1  |              |                      |           |           |                        | 






## Notes
- are vision+encoder based sensing sufficient for localization in indoor environments
- [ROS-Serial](http://wiki.ros.org/rosserial) communicating to embedded hardware (TIVA/Arduino) from ROS using serial communication

## References
- [Sensors for Mobile Robot-1](https://www.sensorsmag.com/components/choosing-best-sensors-for-a-mobile-robot-part-one)
- [Sensors for Mobile Robot-2](https://www.sensorsmag.com/components/choosing-best-sensors-for-a-mobile-robot-part-two)
- [Advice on Robot for Mapping and Navigation](https://www.reddit.com/r/robotics/comments/3mv3q2/robot_mapping_and_navigation_question/)

- 
