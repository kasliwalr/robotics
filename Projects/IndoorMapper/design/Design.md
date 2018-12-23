
[System Module Specification](#system-module-specification)
- [Intelligence Module](#intelligence-module)
- [LIDAR Module](#lidar-module)
- [IMU Module](#imu-module)
- [Servo Module](#servo-module)
- [Power Module](#power-module)



## System Module Specification
Based on the sensor requirements of ROS navigation stack, the system is required to have following sensors. Lidar sensor for range estimation, intertial measurement unit for position estimation over short time periods and encoders (embedded in the servos) for odometry. 
I have divided the overall system based on the lower level hardware. It is also in line with ROS Navigations stack architecture which assumes that it is receving laserscan, odometry information on ROS topics. This architecture choice gives higher degree of flexibility in choice of sensors, gives us better maintainbility as subsystem specific changes will be limited to that subsystem alone and potentially some reusability of these sensor subsystems in other ROS compatible robots. Each subsystem forms a cohesive module, in that it performs a very specific task and is not coupled to other subsystems. 

### Intelligence Module
This is the brain of the robot. This module is responsible for receiving, processing and storing data related to mapping and navigation. This is the software that makes sense of the data coming in, to allow the robot to make a reasonable map. It is also responsible for reporting errors, and generate logs. I may separate that activity in another modules but for now lets keep it here. 

### LIDAR Subsystem
This module is responsible for controlling the LIDAR. It will include such operation such as initialization, starting and stopping the LIDAR, throwing exceptions in case of errors and formatting data for consumption by intelligence module. It may also buffer data if a connection is not available. 

### IMU Module
This module is responsible for controlling the 9-axis IMU 9250. It actions will include IMU initialization, starting and stopping acquisition, recovering from errors, formatting the read data for consumption by intelligence module. 

#### ROS Node API
1. invocation
   - invocation will allow setting the imu node name, imu model to read from and publishing frequency for the odometry message
```
rosrun imu imu_node node_name sensor_model odom_pub_freq
```
   
2. publish odometry
   - message will be of type nav_msgs::Odometry
   - message will be published on topic of name `odom_imu`. 

#### ROS Node Design
1. dependencies
   - nav_msgs::Odometry
   - imu_driver.h
   - roscpp

2. Helper methods
   - generate odometry message from acceleration (m/sec<sup>2</sup>) and gyroscope (radians/sec) measurements
```
void generate_odometry(const double & accMeas, const double & gyrMeas, nav_msgs::Odometry & odom)
```

3. Globals: none
3a. Constants: 
- FS_ACC
- FS_GYR

4. Inputs:
- argv[1]: node name
- argv[2]: imu model type, expected format imu_modeNum
- argv[3]: publish frequency

4a. 

5. Flow
- read command line inputs into argv
- perform inputs type and number error check, if check fails, return 1 with ROS ERROR message, log ROS ERROR
- init Node, with node name - argv[1]
- instantiate node handle

- instantiate Imu object with model type, argv[2]
- advertise publisher of type `nav_msgs::Odometry` using node handle

- instantiate nav_msgs::Odometry object, acceleration and gyro measurement arrays

- while (no termination signal)
  - read acceleration in accelration array
  - read gyro measurements in gyro array
  - generate odometry message using helper method
  - publishe odometry message
  - sleep for loop period

- release imu resources

6. Error Handling
- all Imu APIs will be invoked inside a try catch block. They are expected to return error string on exception. 
- In the catch block, we catch the std::exception, extract the error message, and print to stdout using ROS logging facility at ERROR level. We also log the message using ROS logging facility at ERROR level

#### IMU Driver API

1. namespace
- all imu classes are part of imu_driver namespace


2. instantiation
- instantiates an imu object of specific model type
```
try{
    imu_driver::Imu(string & imu_modelnum);
}
catch(){
 //print and log error information using ROS logging
}
```
- parameter values will be optional, default value is imu_9250
- all public member methods of class Imu return void. They manage error information using exceptions

3. initialize imu
- initializes the imu - selftest followed by calibration 
- optional parameters can be passed. will be decided at a later date
```
Imu.init(param1, param2.....);
```

4. read accelerometer
- read x, y, z acceleration. units m/sec<sup>2</sup>
```
//imu is object of type Imu
imu.readAcc(float *Acc);     //Acc is pointer to first element of a 3 element double array. Acc[0]: x-dir, Acc[1]: y-dir, Acc[2]: z-dir
```

5. read gyroscope
- read x,y,z axis rotational velocity. units radians/sec
```
//imu is object of type Imu
imu.readGyr(float *Gyr);    //Gyr is pointer to first element of a 3 element double array. Gyr[0]: x-rotation, Gyr[1]: y-rotation, Gyr[2]: z-rotation
```

6. shutdown
- aborts connection to imu over i2c. imu is released for use by another process
```
imu.shutdown()
```

#### IMU Driver Design
1. Design Pattern: We will use the [factory pattern](https://www.oodesign.com/factory-pattern.html). 
- ratiale: we need to generate different product (i.e.) objects for each specific type of imu. This is so because the way these objects talk to their specific hardware is dependent on the imu's register api. On the other hand, the way we intend to use these imus in our application will be the same. 

- there we intend to create an interface that the client (i.e. ROS node) will use. We then create a concrete class that implements the interface. There can be multiple such concrete classes, one for each type of imu hardware. 

- we implement a imu_factory. Given an imu model name, it will instantiate an imu object from a set of allowable imu models using its concrete class, and pass the newly created object (casted as the abstract class i.e. the interface) to the client.


2. File types and content:
- there will be a imu_driver.h header file which will contain the method declarations, register address aliases. The header file will have define guards as per [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Header_Files)

- imu_driver.h is included at the end as per [style guideles](https://google.github.io/styleguide/cppguide.html#Name_and_Order_of_Includes)

- we have imu_driver.h that will include the abstract class definitions
- we will have imu9250.h that will have concrete class defintion


3. Namespaces
- using [style guide](https://google.github.io/styleguide/cppguide.html#Namespaces)
- we will wrap our abstract class (interface) and concrete class definitions inside namespaces

### Servo Module
This module is responsible for controlling the Servo. Its actions will include initialization of servo. It will offer different modes of operation such as velocity control, position control, torque control. It will handle servo errors. It is responsible for converting message from intelligence module for consumption by servo, as well as formatting data from servo for consumption by intelligence module. 

### Power Module
The goal of the power module is to supply appropriate voltage and sufficient current for normal operation of the system. The power system should ensure that not only average but peak current needs are met. The power system should also provide power for a suitable amount of time for robot to operate nominally. The power system should also ensure protection of Lipo battery from over-discharge to prevent premature replacement of batteries. It should ideally monitor power consumption of the system for metering purposes. 

#### Power Budget

|Component|Qty|Avg Idle I (mA)|Avg Max I (mA)|Peak I (mA)|Voltage (V)|V<sub>ripple</sub>(V/ %)|Total Avg Max I (mA)|
|---------|---|---------------|----------------------|-----------|-----------|------------------------|------------|
|[R-Pi3](https://www.pidramble.com/wiki/benchmarks/power-consumption)|1  |260            |730                   |?          | 5.1       |+/- 0.3V                     | 730|
|[R-Pi3 + CAM](https://raspi.tv/2016/how-much-power-does-raspberry-pi3b-use-how-fast-is-it-compared-to-pi2b)|1|260            |850                   |?          |5.1          |+/-0.3V                      |850|
|[Dynamixel XL430-W250-T](http://support.robotis.com/en/)|2     |52                    |          |?       |11.1                       |?|>100 ?|
|[LiDAR](http://bucket.download.slamtec.com/8e7a1f4490a235717b43fccaf7dcae325dda7dc8/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.1_en.pdf)    |1   |250           |500                   |700        |5          |0.02                    |500|

[RPi Official](https://www.raspberrypi.org/documentation/faqs/#pi-power) </br>

#### Power Distribution 
The goal of the power distribution system to to supply all components of the system power at specified voltages and current. I will provide a brief description of the power distribution in Turtlebot3 Burger modified version that I am making. Specifically, I will list the components and elaborate a bit on the reasons for choosing the components. 

Component choice was mostly driven by budget, the original Robotis platform was > $500.00, this was unnecessarily expensive for me. I already had a few parts and found a few other unnecessary for my initial prototype. So here goes.

1. Battery: Robotis offers a 11.1 V LiPo battery, 1800mAh, 5C with protection circuitry, price $50.00. On the market. It weight around 106 gms and is 88cm long. I could find similar battery with higher current rating of 2500mAh, 5C. This was 30 gms heavier and 10cm longer but at a price of $25.00. One can add a $10.00 battery protection ckt board, making is overall $15.00 cheaper. It could have been $25.00 cheaper but for the shipping cost. 
2. Step Down Convertors: Two convertors were needed for the system. First one converts 11.1V to 5VDC at a max of 5A, this would supply to Raspberry Pi, and LiDAR motor. The second one also is a 12V to 5V DC convertor but with low ripple. This would supply power exclusively to the LiDAR scanner
3. Wiring:
The battery will supply power to three area connected in star-topology. 
- first will be the two dynamixel servos will be connected to each other using TTL cable. One one end the ttl cable will be connected to battery power. 
- second will be 12V-5V DC convertor supplying power to RPi, and Lidar motor. It will be placed on the second floor close to its respective sinks
- third will be 12V-5V SC convertor (low ripple), supplying power to LiDAR scanner, it will be placed on the 3rd floow, close to its respective sink

## Hardware Specification

![hw schematic](images/hw_schematic.jpg)

## Software Specification
![call_graph](images/call_graph.jpg)

## BOM 
|Part Name|Part Number|Vendor| Vendor Part Number|Qty|Unit Price ($)|Total Price ($)|
|---------|-----------|------|-------------------|---|--------------|---------------|
|Raspberry Pi 3B|IM00001|[CanaKit](https://www.canakit.com/raspberry-pi-3-model-b.html?cid=usd&src=raspberrypi)|SKU: PI3|1|35.00|35.00|
|Lipo SHIM|IM00002|[Pimoroni](https://shop.pimoroni.com/products/lipo-shim)|PIM185|1|9.95|9.95|
|Switch, PushButton|IM00003|[Polulu](https://www.pololu.com/product/2808)|2808|1|3.95|3.95|
|Battery, Li-Poly|IM00004|[HobbyKing](https://hobbyking.com/en_us/turnigy-nano-tech-2500mah-3s1p-5-10c-transmitter-lipo-pack-futaba-6ex-and-3pks.html)|SKU 9210000037|1|17.98|17.98|
|Battery Protection PCB|IM00005|[Diymore](https://www.diymore.cc/products/3s-12v-10a-18650-bms-charger-module-li-ion-lithium-battery-protection-board)|SKU 012245|1|1.99|1.99|
|USB to Serial, FT232RL|IM00006|[Sparkfun](https://www.sparkfun.com/products/12731)|BOB-12731|1|15.95|15.95|
|Inverter, Hex, SN74HC04|IM00007|[Digikey](https://www.digikey.com/product-detail/en/texas-instruments/SN74HC04N/296-1566-5-ND/277212)|296-1566-5-ND|1|0.55|0.55|
|Buffer Gate, 3-STATE, SN74HCT125|IM00008|[Digikey](https://www.digikey.com/product-detail/en/texas-instruments/SN74HCT125N/296-8386-5-ND/376860)|296-8386-5-ND|1|0.40|0.40|
|Connector Header, 3 pin, B3B-EH-A|IM00009|[Digikey](https://www.digikey.com/product-detail/en/jst-sales-america-inc/B3B-EH-A-LF-SN/455-1612-ND/926521)|455-1612-ND|1|0.19|0.19|
|Servo Motor|IM00010|[Robotis](http://www.robotis.us/dynamixel-xl430-w250-t/)|XL430-W250T|2|49.90|99.8|
|Intertial Measurement Unit 9250|IM00011|[Sparkfun](https://www.sparkfun.com/products/13762)|SEN-13762|1|14.95|14.95|
|LIDAR|IM00012|[Seeed Studio](https://www.seeedstudio.com/RPLiDAR-A1M8-360-Degree-Laser-Scanner-Kit-12M-Range-p-3072.html)|SKU 110991065|1|99.00|99.00|
|Regulator, 12V-5VDC, Step Down|IM00013|[Polulu](https://www.pololu.com/product/2851)|2851|1|14.95|14.95|

### Part Naming Scheme
IMxxyyy: IM - indoor mapper, xx - 2 digit code for part type (00-Electrical parts), yyy - 3 digit code indicating part number for a particular type


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
- [XL430-W250T Control Table](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-description)
- [Dynamixel SDK C++ library Setup](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux)
- [Dynamixel Protocol 2.0](http://emanual.robotis.com/docs/en/dxl/protocol2/)
- [RPiLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)
- [RpiLIDAR User Manual](docs/slamtec_lidar_user_man.pdf)
- [RpiLIDAR Protocol](docs/slamtec_lidar_interface_protocol.pdf)

