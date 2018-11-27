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

### Dynamixel
Keywords: Dynamixel SDK C/C++ API, Dynamixel Protocol 2.0, USB2Dynamixel, U2D2, Protocol 2.0, Control Table, EEPROM Area, RAM Area
On linux, Dynamixel SDK is available as shared library (.so) as well as source code. Examples are also available

#### Overview

ROBOTIS Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The Dynamixel API requires programming in C/C++. We wull use the recommended Protocol 2.0 for communicating with the X-series servo. We will test the SDK on Ubuntu 16.04 LTS running on Intel X86 and Raspbian running on Raspberry Pi 3. We will use FT232RL serial board from Sparkfun with custom circuit to perform half-duplex communication with the servo. Finally we will also test the dynamixel using ROS C++ package on Ubunutu or Raspbian. 


##### Installation

We cloned the dynamixel repository on github: https://github.com/ROBOTIS-GIT/DynamixelSDK
```
> git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
We also installed the ROS library
```
>sudo apt-get install ros-kinetic-dynamixel-sdk      # we are using ROS-Kinetic
```
Before we start exploring the library, we will install hardware driver for FT232L


FDTI devices have two types of driver - virual COM port driver (VCP) and D2XX API driver. FTDI VCP driver is build into the Linux kernel. In Linux, VCP drivers will appear as /dev/ttyUSBx.

Verify built-in COM port
1. Plug in FTDI device
2. In terminal window
```
> dmesg | grep FTDI
# The following output should be generated
[334680.372193] usb 2-1.2.2: Manufacturer: FTDI
[334681.410183] usbserial: USB Serial support registered for FTDI USB Serial Device
[334681.410249] ftdi_sio 2-1.2.2:1.0: FTDI USB Serial Device converter detected
[334681.410899] usb 2-1.2.2: FTDI USB Serial Device converter now attached to ttyUSB0
[334683.068937] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
[334693.939291] usb 2-1.2.2: Manufacturer: FTDI
[334693.942056] ftdi_sio 2-1.2.2:1.0: FTDI USB Serial Device converter detected
[334693.942666] usb 2-1.2.2: FTDI USB Serial Device converter now attached to ttyUSB0
[334696.892600] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
[334718.771810] usb 2-1.2.2: Manufacturer: FTDI
[334718.774809] ftdi_sio 2-1.2.2:1.0: FTDI USB Serial Device converter detected
[334718.775587] usb 2-1.2.2: FTDI USB Serial Device converter now attached to ttyUSB0
[334889.403163] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
[334903.601907] usb 2-1.2.2: Manufacturer: FTDI
[334903.604827] ftdi_sio 2-1.2.2:1.0: FTDI USB Serial Device converter detected
[334903.605279] usb 2-1.2.2: FTDI USB Serial Device converter now attached to ttyUSB0
[334915.771189] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
[334925.854973] usb 2-1.2.2: Manufacturer: FTDI
[334925.858182] ftdi_sio 2-1.2.2:1.0: FTDI USB Serial Device converter detected
[334925.858748] usb 2-1.2.2: FTDI USB Serial Device converter now attached to ttyUSB0
```

### Dynamixel SDK 

#### File Structure
Example codes are using either C/C++. Dynamixel SDK repo folder contains folders for each supported API language. We are interested in C++. 

In c++ folder, you'll find cpp source files, header files, build files and example codes. 


![sdl lib structure](images/sdk_library_struct.png)

Though the PortHandler which handles system communication environment is separated in three OS, the Linux, the macOS and the Windows, the other sources are made to be able to be cross-compiled. We use a separate Makefile for each environment to build the src code. 

There is example code in `example` folder. For us, the subfolder `protocol2.0` containst the relevant example code. There are platform specific Makefiles in example/protocol2.0 to build the example code. 



The dynamixel SDK expect the device name to be `/dev/ttyUSB0` for linux system, we have verified that to be the case. 

Note: The SDK has been tested on Ubuntu 16.04 and Raspbian Jessie, which we are using. 

#### Library Setup
For detailed instruction, see [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux)

1. Compiler: GNU gcc ver. 5.4.0 20160609 or higher
```
> gcc -v
gcc version 5.4.0 20160609 (Ubuntu 5.4.0-6ubuntu1~16.04.10) 
```
2. Builder
```
> sudo apt-get install build-essential
```
3. Dependent Packages: packages needed for cross-compiling
```
> sudo apt-get install gcc-multilib g++-mulitlib
```
4. Build the library: navigate to `linux64` subdirectory in `build`
```
> pwd
/home/rk/repos/DynamixelSDK/c++/build/linux64
```
5. Invoke make: This will generate a shared library file, `libdxl_x64_cpp.so`
6. Install the library: 
```
> sudo make install
```
#### Running the Sample Code
1. Navigate to example code for linux64
```
> pwd   
/home/rk/repos/DynamixelSDK/c++/example/protocol2.0/read_write/linux64
```
2. Invoke make
```
> make
> ls
Makefile  read_write
```
3. Make the port available to be used
> sudo chmod a+rw /dev/ttyUSB0
```
4. Run the executable
```
> ./read_write
```
Succeeded to open the port!
Succeeded to change the baudrate!
Dynamixel has been successfully connected 
```

Robotis also provides code for interactive control through `dxl_monitor`. Navigate to dxl_monitor folder
```
> pwd
/home/rk/repos/DynamixelSDK/c++/example/dxl_monitor
```
Navigate to `linux64` folder, and make
```
> pwd
/home/rk/repos/DynamixelSDK/c++/example/dxl_monitor/linux64
> make
> ./dxl_monitor            # run to go into interactive mode
> ?                        # list command help
```
Use XL430-250T parameters as specified [here](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-description)


### RPiLIDAR

### RPLIDAR SDK
We will detail use of RPLIDAR SDK on Ubuntu 16.04 LTS. 

1. Clone the open source SDK from github repository
```
git clone https://github.com/Slamtec/rplidar_sdk.git
```

### Cleaning up Raspberry Pi
Raspberry Pi has lot of bloatware which takes up space on the SD card. We will show steps to remove unnecessary software installed on Raspberry Pi
```
> sudo apt-get purge wolfram-engine
> sudo apt-get clean; sudo apt-get autoremove
> sudo apt-get purge libreoffice*
> sudo apt-get clean; sudo apt-get autoremove
> 
```


### Running ROS on multiple machines


#### Configuring Raspberry Pi for Internet over Wifi
For our purposes, we would like to remotely configure / control Turtlebot from our PC. The onboard Raspberry Pi is capable of connectivity over Ethernet and Wifi. We would configure its Wifi, so that we can connect to it over our home Wifi network. 
In future, we could use this same home Wifi router to communicate to Turtlebot and view its status over the internet from anywhere in the world. 

Based on the guidelines here, we configured the wireless interface first. We configured it for home network, with a netmask of `255.255.255.0` and gateway of `192.xxx.xxx.xxx`. The ip address was statically assigned. On Raspbian, interface are configured by editing the /etc/network/interfaces file. Our file looked as follows

```
# ethernet config
auto eth0
iface eth0 inet static
    address 192.xxx.xxx.xxx
    netmask 255.255.255.0
    gateway 192.xxx.xxx.xxx

# wifi config
auto wlan0
allow-hotplug wlan0
iface wlan0 inet static
    address 192.xxx.xxx.xxx
    netmask 255.255.255.0
    gateway 192.xxx.xxx.xxx
    wpa-conf /path_to_wpa_supplicant/wpa_supplicant.conf

source-directory /path_to_interfaces.d/interfaces.d
```

Then we configured the wpa_supplicant.conf. This file is read by the wpa_supplicant daemon, to perform wpa_client actions on the host machine. 

```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US

network={
        ssid="My_SSID"
        scan_ssid=1
        psk="pass_phrase"
        key_mgmt=WPA-PSK
}
```
We had to provide the `scan_ssid=1` field, because our SSID is hidden. The result of the fields are standard.

To test the Wifi, the ethernet was disconnected, and Pi was rebooted. On reboot, we did an ssh into Pi from our PC. 
```
> ssh pi@192.xxx.xxx.xxx
pi@192.xxx.xxx.xxx's password:
```
This indicates that we can communicate to Raspberry Pi over the home-WLAN. 

#### Communicating over multiple Machines using ROS
We intend to run some nodes on Raspberry Pi, and computation intensive nodes and ROS master on PC. To verify, that we could do so, we tested communication between talker and listener nodes running on Raspberry PI and PC respectively. 

It is assumed that ROS has already been installed on Raspberry Pi. We followed the ROS tutorial provided [here](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

1. Start ROS on PC
```
# start ROS master on PC
> roscore
# configure ROS_MASTER_URI
> export ROS_MASTER_URI=http://pc_ip:11311
# run listener
> rosrun rospy_tutorials listener.py
```
2. Start ROS on Raspberry Pi
```
# ssh to Pi
> ssh pi@192.xxx.xxx.
# Configure ROS_MASTER_URI
> export ROS_MASTER_URI=http://pc_ip:11311
#configure ROS_IP
> export ROS_IP=Raspi_wlan_ip
# run talker
> rosrun rospy_tutorials talker.py
```
This should start showing messages sent by talker to listener, on the PC. If successful, we are done. Now we have a distributed ROS system, where master runs on PC, and other ROS nodes run on either PC or Raspberry Pi. 






### Hardware Installation


Circuit Diagram image needed
Picture of the actual connection




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


### ROS Installation Raspberry Pi

[ROS Kinetic Installation on Raspberry Pi Stretch](http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi?action=fullsearch&context=180&value=linkto%3A%22ROSberryPi%2FInstalling+ROS+Kinetic+on+the+Raspberry+Pi%22)


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

