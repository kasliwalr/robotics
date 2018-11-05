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

|Component|Qty|Avg Idle I (mA)|Avg Max I (mA)|Peak I (mA)|Voltage (V)|V<sub>ripple</sub>(V/ %)|Total Avg Max I (mA)|
|---------|---|---------------|----------------------|-----------|-----------|------------------------|------------|
|[R-Pi3](https://www.pidramble.com/wiki/benchmarks/power-consumption)|1  |260            |730                   |?          | 5.1       |+/- 0.3V                     | 730|
|[R-Pi3 + CAM](https://raspi.tv/2016/how-much-power-does-raspberry-pi3b-use-how-fast-is-it-compared-to-pi2b)|1|260            |850                   |?          |5.1          |+/-0.3V                      |850|
|TIVA Launchpad|1|NA          |45<sup>a</sup>        |           |5V         |+/0.25V                 |45|
|[Dynamixel XL430-W250-T](http://support.robotis.com/en/)|2     |52                    |?          |11.1       |?                       |?|>100 ?|
|[LiDAR](http://bucket.download.slamtec.com/8e7a1f4490a235717b43fccaf7dcae325dda7dc8/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.1_en.pdf)    |1   |250           |500                   |700        |5          |0.02                    |500|
|IR Sensor|5   |?             |40                    |           |5          |+/-0.5                  |200|
|Nvidia TX1|1  |?             |?                     |?          |12V        |?                       |?| 

[RPi Official](https://www.raspberrypi.org/documentation/faqs/#pi-power) </br>
a: current consumption chart in [TM4C123GH6PM datasheet](http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf), @80MHZ, all peripherals enabled. Current consumption by specific attached peripherals, or in action is not included. 


## Hardware Schematic

#### Dynamixel -- RPi 
Driving Dyanmixel using Raspberry Pi.

Keywords: Dynamixel SDK C/C++ API, Dynamixel Protocol 2.0, USB2Dynamixel, U2D2, Protocol 2.0, Control Table, EEPROM Area, RAM Area
On linux, Dynamixel SDK is available as shared library (.so) as well as source code. Examples are also available




### Power Distribution 
The goal of the power distribution system to to supply all components of the system power at specified voltages and current. I will provide a brief description of the power distribution in Turtlebot3 Burger modified version that I am making. Specifically, I will list the components and elaborate a bit on the reasons for choosing the components. 

Component choice was mostly driven by budget, the original Robotis platform was > $500.00, this was unnecessarily expensive for me. I already had a few parts and found a few other unnecessary for my initial prototype. So here goes.

1. Battery: Robotis offers a 11.1 V LiPo battery, 1800mAh, 5C with protection circuitry, price $50.00. On the market. It weight around 106 gms and is 88cm long. I could find similar battery with higher current rating of 2500mAh, 5C. This was 30 gms heavier and 10cm longer but at a price of $25.00. One can add a $10.00 battery protection ckt board, making is overall $15.00 cheaper. It could have been $25.00 cheaper but for the shipping cost. 
2. Step Down Convertors: Two convertors were needed for the system. First one converts 11.1V to 5VDC at a max of 5A, this would supply to Raspberry Pi, and LiDAR motor. The second one also is a 12V to 5V DC convertor but with low ripple. This would supply power exclusively to the LiDAR scanner
3. Wiring:
The battery will supply power to three area connected in star-topology. 
- first will be the two dynamixel servos will be connected to each other using TTL cable. One one end the ttl cable will be connected to battery power. 
- second will be 12V-5V DC convertor supplying power to RPi, and Lidar motor. It will be placed on the second floor close to its respective sinks
- third will be 12V-5V SC convertor (low ripple), supplying power to LiDAR scanner, it will be placed on the 3rd floow, close to its respective sink

![wiring diagram](images/power_wiring.png)


2. Robotis OpenCR1.0: THis board has a cortex m7 microcontroller and also serves as a power distribution board but its $179.00, one-third the cost of the turtlebot. This board handles communication with Robotis servos using either RS485, UART. So it will interface with any Robotis Servo. I was interested only in using the X series, XL-430 W250T servo. An alternative scheme was to use the U2D3 USB to TTL convertor sold by Robotis. This allows the RPi to communicate to the servo using USB protocol, and interfaces with the half-duplex TTL interface on the servo. It cost $50.00. Most USB to Serial convertors on market, dont do half duplex TTL. So, I search for some hacking solutions, I found a USB to UART convertor from sparkfun (FDTI chip) for $17.00, and I could add some interfacing circuitry to convert it to half-duplex TTL. Overall cost to around $20.00


UART to half-duplex: https://devtalk.nvidia.com/default/topic/1039093/half-duplex-uart-from-dev-ttyths2/




## Notes
- are vision+encoder based sensing sufficient for localization in indoor environments
- [ROS-Serial](http://wiki.ros.org/rosserial) communicating to embedded hardware (TIVA/Arduino) from ROS using serial communication

## References
- [Sensors for Mobile Robot-1](https://www.sensorsmag.com/components/choosing-best-sensors-for-a-mobile-robot-part-one)
- [Sensors for Mobile Robot-2](https://www.sensorsmag.com/components/choosing-best-sensors-for-a-mobile-robot-part-two)
- [Advice on Robot for Mapping and Navigation](https://www.reddit.com/r/robotics/comments/3mv3q2/robot_mapping_and_navigation_question/)
- [UART to half duplex1](https://devtalk.nvidia.com/default/topic/1039093/half-duplex-uart-from-dev-ttyths2/)
- [UART to hald duplex2](https://wot.lv/using-dynamixels-roboplus-without-usb2dynamixel.html)
- [Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- Connectors
  - [Connectors Dynamixel](http://support.robotis.com/en/product/actuator/dynamixel/dxl_connector.htm) 
  - [Connectors 
- 
