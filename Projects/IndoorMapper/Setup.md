[Software Setup](#software-setup)
- [Raspbian Stretch Installation on Raspberry Pi3B](#raspbian-stretch-installation-on-raspberry-pi-3b)
  - [Cleaning up Raspberry Pi](#cleaning-up-raspberry-pi)
- [ROS Installation](#ros-installation)
  - [ROS Installation on PC Running Ubuntu 16.04 LTS](#ros-installation-on-pc-running-ubuntu-1604-lts)
  - [ROS Installation on Raspberry Pi3B Running Raspbian Stretch](#ros-installation-on-raspberry-pi3b-running-raspbian-stretch)
    - [ROS-Kinetic Installation on Raspbian Stretch](#ros-kinetic-installation-on-raspbian-stretch)
  - [Configuring ROS for PC and Raspberry Pi3B](#configuring-ros-for-pc-and-raspberry-pi3b)
    - [Configuring Raspberry Pi3B for Connection over Wifi](#configuring-raspberry-pi3b-for-connection-over-wifi)
    - [Configuring ROS for communication over multiple machines](#configuring-ros-for-communication-over-multiple-machines)
- [Dynamixel Installation](#dynamixel-installation)
  - [Dynamixel SDK CPP Library Installation](#dynamixel-sdk-cpp-library-installation)
    - [Dynamixel SDK File Structure](#dynamixel-sdk-file-structure)
    - [Library Setup on Ubuntu and Raspbian Stretch](#library-setup-on-ubuntu-and-raspbian-stretch)
    - [FTDI FT232 Driver Installation](#ftdi-ft232-driver-installation)
    - [Running the Sample Code](#running-the-sample-code)
  - [ROS Dynamixel Workbench Installation](#ros-dynamixel-workbench-installation)
    - [Dynamixel Workbench Installation on Raspberry Pi3B and PC](#dynamixel-workbench-installation-on-raspberry-pi3b-and-pc)
    - [Testing Dynamixel Workbench installation on Raspberry Pi](#testing-dynamixel-workbench-installation-on-raspberry-pi)
    - [Testing Distributed Operation of Dynamixel Workbench](#testing-distributed-operation-of-dynamixel-workbench)
- [RPLIDAR Setup](#rplidar-setup)
  - [RPLIDAR SDK Installation](#rplidar-sdk-installation)
  - [RPLIDAR ROS Installation on Raspberry Pi](#rplidar-ros-installation-on-raspberry-pi)
- [IMU 9250 Setup](#imu-9250-setup)
  - [IMU 9250 ROS Installation on Raspberry Pi](#imu-9250-ros-installation-on-raspberry-pi)
  
  
# Software Setup
The software for the Indoor Mapper is deployed two machines. One is a PC running Ubuntu 16.04 LTS and other is Raspberry Pi 3 running Raspbian Stretch. We will detail the installation of hardware drivers as well ROS framework on both machines. 

## Raspbian Stretch Installation on Raspberry Pi 3B

You would need to have an installation media (SD Card with >= 16GB) to install Raspbian on Raspberry Pi. You could buy a preinstallated installation media. I prepared the installation media myself with instructions [here](https://www.raspberrypi.org/documentation/installation/noobs.md) way to install Raspbian 

There are instructions for Window, Linux and MacOS users to prepare the NOOBS. It is recommended to install [Raspbian Stretch Lite](https://www.raspberrypi.org/downloads/raspbian/), if you are unable to find Raspbian Lite when using NOOBS, then you will have to prepare the Raspbian image. 

For those who have already have the Raspbian Stretch installed, it is recommended to clean it up before continuing (see cleanup instruction below). 

### Cleaning up Raspberry Pi
Raspbian has lot of bloatware which takes up space on the SD card. Before we install ROS, we will clean up the bloatware. Below are steps to remove wolfram-engine, libreoffice suite, minecraft and sonic-pi. 
```
> sudo apt-get purge wolfram-engine
> sudo apt-get clean; sudo apt-get autoremove
> sudo apt-get purge libreoffice*
> sudo apt-get clean; sudo apt-get autoremove
> sudo apt-get purge minecraft-pi
> sudo apt-get clean; sudo apt-get autoremove
> sudo apt-get purge sonic-pi
> sudo apt-get clean; sudo apt-get autoremove
```



## ROS Installation
We intend to run ROS nodes on both the PC and Raspberry Pi. Below are details for installation of ROS. 


### ROS Installation on PC Running Ubuntu 16.04 LTS
Follow instruction provided [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) here for installing ROS Kinetic Kame full desktop version. 
```
> sudo apt-get install ros-kinetic-desktop-full
```
For installing additional packages, do the following
```
> sudo apt-get install ros-kinetic-PACKAGE
```
Don't forget to configure the .bashrc file to automatically load the ROS environment variables everytime a new session window is opened. 

### ROS Installation on Raspberry Pi3B Running Raspbian Stretch

#### ROS-Kinetic Installation on Raspbian Stretch
We followed instruction provided [here](http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi?action=fullsearch&context=180&value=linkto%3A%22ROSberryPi%2FInstalling+ROS+Kinetic+on+the+Raspberry+Pi%22)

We installed the base version of ROS-kinetic. This is the bare minimum. We didn't want to install the full version because we do not know at this point which packages we will need. We will mostly likely not run navigation/mapping tasks on Raspberry Pi because of the computational load. We will install packages on as-needed basis. 

### Configuring ROS for PC and Raspberry Pi3B

#### Configuring Raspberry Pi3B for Connection over Wifi
For our purposes, we would like to remotely configure / control IndoorMapper from our PC. The onboard Raspberry Pi3B is capable of connectivity over Ethernet and Wifi. We would configure its Wifi, so that we can connect to it over our home Wifi network. 
In future, we could use this same Wifi router to communicate to IndoorMapper and view its status over the internet from anywhere in the world. 

Based on the guidelines here, we configured the wireless interface first. We configured it for home network, with a netmask of `255.255.255.0` and gateway of `192.xxx.xxx.xxx`. The ip address was statically assigned. On Raspbian, interfaces are configured by editing the /etc/network/interfaces file. Our file looked as follows

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
We had to provide the `scan_ssid=1` field, because our SSID is hidden. The rest of the fields are standard. To test the Wifi, the ethernet was disconnected, and Pi was rebooted. On reboot, we did an ssh into Pi from our PC. 
```
> ssh pi@192.xxx.xxx.xxx
pi@192.xxx.xxx.xxx's password:
```
This indicates that we can communicate to Raspberry Pi over the home-WLAN. 

#### Configuring ROS for Communication over multiple machines
I intend to run sensor driver nodes on Raspberry Pi, and computation intensive nodes and ROS master on PC. To verify, that we could do so, we tested communication between talker and listener nodes running on Raspberry PI and PC respectively. The talker publishes messages while the listerner listens is subscribed to the topic and prints them as it receives them. If all goes well, the listerner should be printing on PC console messages sent by talker on Raspberry Pi. 

We followed the ROS tutorial provided [here](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

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
> export ROS_IP=Raspi_wlan_ip             # this should the static ip address you assigned you to WLAN interface on Raspberry Pi
# run talker
> rosrun rospy_tutorials talker.py
```
This should start showing messages sent by talker to listener, on the PC. If successful, we are done. Now we have a distributed ROS system, where master runs on PC, and other ROS nodes run on either PC or Raspberry Pi. 

## Dynamixel Installation
Dynamixel SDK is a software development kit that allows you to develop applications for controlling the dynamixel servos using a variety of APIs (C++, Python, Matlab, LabVIEW etc). You could develop application of various platforms as well - Raspbian running on Raspberry Pi, Linux on any PC etc. On a PC or raspberry Pi, the SDK enables packet communication to the servos by invoking the USB driver. A USB to UART connection between the PC and the servos, then serializes the packets for servos. 

We will be developing applications for servos on both PC and Raspberry Pi3B. Although the servos will run on rasberry Pi, we may need to connect them to the PC directly for application testing purposes. We will be using C++ based API for controlling the servos. Any ROS nodes will be developed using SDK C++ libraries and roscpp. 

In these setup instruction, we will work with both the Dynamixel C++ SDK as well as ROS tools for controlling dynamixel. We have detailed the installation and usage examples for both Dynamixel SDK and ROS tools. 

### Dynamixel SDK CPP Library Installation 

#### Dynamixel SDK File Structure
Example codes are using either C/C++. Dynamixel SDK repo folder contains directories for each supported API language. We are interested in C++. 

In `c++` folder, you'll find cpp source files, header files, build files and example codes. 
![sdl lib structure](images/sdk_library_struct.png)

The Makefile is contained in the `build` directory. We use a separate Makefile for each environment to install the library in - use the `linux64` Makefile for Ubuntu16.04LTS running on 64-bit PC and `linux_sbc` Makefile for Raspbian Strectch running on Raspberry Pi3B. 

Once library installation is complete, we build and run the c++ example code. We run two examples - `dxl_monitor` which is a interactive program to configure the servos and `read_write` which sets position goal for servos and reads position values from it. The example code is in `example` folder. Since we are going to use the recommended protocol2.0 for communication, we will use the code in `protocol2.0` folder. There are platform specific Makefiles in protocol2.0 folder for running each example. 

The dynamixel SDK expect the device name to be `/dev/ttyUSB0` for linux system, we have verified that to be the case. Later we'll see how we can tweak the SDK to handle dyamixel servos being on other ports. 

Note: The SDK has been tested on Ubuntu 16.04 and Raspbian Stretch, which we are using. 

#### Library Setup on Ubuntu and Raspbian Stretch
Since Raspbian and Ubuntu are linux distribution, the installation procedure is mostly same.
For detailed instruction, see [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux). Below are steps for setting up library on Ubuntu 16.04 LTS. The SDL C++ libraries have certain dependencies that need to be installed before we install the library. 

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
4a. Build the library specific to Linux64 configuration. Navigate to `linux64` subdirectory in `build`
```
> pwd
/home/rk/repos/DynamixelSDK/c++/build/linux64
```
4b. Build the library specific to Raspbian Strectch configuration. Navigate to `linux_sbc` subdirectory in `build`
```
> pwd
/home/rk/repos/DynamixelSDK/c++/build/linux_sbc
```

5. Invoke make: This will generate a shared library file, `libdxl_x64_cpp.so` on Ubuntu16.04LTS and `libdxl_sbc_cpp.sp` on Raspbian Stretech
6. Install the library: 
```
> sudo make install
```
Before running sample code make sure that servos are connected through the USB to UART converter to the PC or Raspberry Pi. I custom build by USB to half-duplex UART converter using FTDI USB to full-duplex UART convertor (FTDI FT232). The instructions below use detail its installation

#### FTDI FT232 Driver Installation
1. Install hardware driver for FT232. FDTI devices have two types of driver - virual COM port driver (VCP) and D2XX API driver. FTDI VCP driver is build into the Linux kernel. On Linux, VCP drivers will appear as `/dev/ttyUSBx`. Whether you are installting on Ubuntu or Raspbian, it should come preinstallated 

2. Verify built-in VCP driver
- Plug in FTDI device
- In terminal window
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
It should say "FTDI USB Device now attached/detected" or something like that. 

#### Running the Sample Code
Assuming you have run installed the Dynamixel SDK C++ library and FTDI FT232 drivers, you can now test the sample programs. I tested two sample program, first is dxl_monitor which when run allows user to configure servos using command line interface. I ran this first to configure the servos ID and baud rate. If you are daisy chaining servos, you need to configure them for same baud rate and unique IDs. 

**dxl_monitor**</br>
Robotis also provides code for interactive control through `dxl_monitor`. Navigate to dxl_monitor folder
```
> pwd
/home/rk/repos/DynamixelSDK/c++/example/dxl_monitor
```
Navigate to `linux64` folder on Ubuntu and `linux_sbc` on Raspbian, and build using `make`
```
> pwd
/home/rk/repos/DynamixelSDK/c++/example/dxl_monitor/linux64
> make
> ./dxl_monitor            # run to go into interactive mode
> ?                        # list command help
                    .----------------------------.
                    |  DXL Monitor Command List  |
                    '----------------------------'
 =========================== Common Commands ===========================
 
 help|h|?                    :Displays help information
 baud [BAUD_RATE]            :Changes baudrate to [BAUD_RATE] 
                               ex) baud 57600 (57600 bps) 
                               ex) baud 1000000 (1 Mbps)  
 exit                        :Exit this program
 scan                        :Outputs the current status of all Dynamixels
 ping [ID] [ID] ...          :Outputs the current status of [ID]s 
 bp                          :Broadcast ping (Dynamixel Protocol 2.0 only)
 
 ==================== Commands for Dynamixel Protocol 1.0 ====================
 
 wrb1|w1 [ID] [ADDR] [VALUE] :Write byte [VALUE] to [ADDR] of [ID]
 wrw1 [ID] [ADDR] [VALUE]    :Write word [VALUE] to [ADDR] of [ID]
 rdb1 [ID] [ADDR]            :Read byte value from [ADDR] of [ID]
 rdw1 [ID] [ADDR]            :Read word value from [ADDR] of [ID]
 r1 [ID] [ADDR] [LENGTH]     :Dumps the control table of [ID]
                               ([LENGTH] bytes from [ADDR])
 reset1|rst1 [ID]            :Factory reset the Dynamixel of [ID]
 
 ==================== Commands for Dynamixel Protocol 2.0 ====================
 
 wrb2|w2 [ID] [ADDR] [VALUE] :Write byte [VALUE] to [ADDR] of [ID]
 wrw2 [ID] [ADDR] [VALUE]    :Write word [VALUE] to [ADDR] of [ID]
 wrd2 [ID] [ADDR] [VALUE]    :Write dword [VALUE] to [ADDR] of [ID]
 rdb2 [ID] [ADDR]            :Read byte value from [ADDR] of [ID]
 rdw2 [ID] [ADDR]            :Read word value from [ADDR] of [ID]
 rdd2 [ID] [ADDR]            :Read dword value from [ADDR] of [ID]
 r2 [ID] [ADDR] [LENGTH]     :Dumps the control table of [ID]
                               ([LENGTH] bytes from [ADDR])
 reboot2|rbt2 [ID]           :reboot the Dynamixel of [ID]
 reset2|rst2 [ID] [OPTION]   :Factory reset the Dynamixel of [ID]
                               OPTION: 255(All), 1(Except ID), 2(Except ID&Baud)
```
To change any parameter, you need to write onto the specific servo's (identified by its ID) ROM or RAM memory using protocol2.0 commands. To change the ID you need to write to memory location 7 any value between 0-255. EEPROM is write protected if `Torque_Enable` value at memory location 64 in RAM. Before you can change ID of servo, disable `Torque_Enable` by writing a zero, then change id and finally set `Torque_Enable` to disable write to EEPROM. I am using Dynamixel Servo XL430-250T, so I am using the  its [control table](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-description)


**read_write**</br>
The example codes are protocol specific and system specific. This function writes position values to the servo's microcontroller which makes it move in specific direction. I had to tweak the source code for the example, the exact sequence is found here. 

1a. Navigate to example code for linux64 on Ubuntu
```
> pwd   
/home/rk/repos/DynamixelSDK/c++/example/protocol2.0/read_write/linux64
```
1b. Navigate to example code for linux_sbc on Raspbian
```
> pwd   
/home/rk/repos/DynamixelSDK/c++/example/protocol2.0/read_write/linux_sbc
```

2. Invoke make
```
> make
> ls
Makefile  read_write
```
3. Make the USB port available to be used without root permissions
> sudo chmod a+rw /dev/ttyUSB0
```
4. Run the executable
```
> ./read_write                   # the servo should rotate back and forth
```
Succeeded to open the port!
Succeeded to change the baudrate!
Dynamixel has been successfully connected 
```
In read_write.cpp, make sure these parameters are defined as below
```
#define ADDR_PRO_TORQUE_ENABLE          64 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
```

One also needs to pay attention to definition of following parameters
```
#define DXL_ID                          2                   // Dynamixel ID: 1
#define BAUDRATE                        57600               // should be same as that set using dxl_monitor
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
```
If you choose you use a different USB port, set it here, for ex. to `/dev/ttyUSB1` (with the double quotes)

### ROS Dynamixel Workbench Installation 
Dynamixel workbench is a ROS metapackage that contains a bunch of ROS packages consisting of both ROS nodes and libraries. The workbench packages that are essential for building any ROS APIs for dynamixel are `dynamixel_workbench_toolbox` and `dynamixel_workbench_msgs`. There are two additional packages that are good starter packages for controlling and operating the servos. These are `dynamixel_workbench_controllers` and `dynamixel_workbench_operators`. They use the previous two packages along with `roscpp` API for develop nodes that offer ROS services and exchange information over ROS topics, as well as talk to servos.  We will learn how to install selected packages from `dynamixel_workbench` metapackage on both Ubuntu and Raspberry Pi and run a velocity control application in a distrbuted manner, so that we can control the servo from PC through Raspberry Pi. 

#### Dynamixel Workbench Installation on Raspberry Pi3B and PC
We are interested in installing the following 5 ROS packages for dynamixel - `dynmixel_sdk`, `dynamixel-workbench-toolbox`, `dynamixel-workbench-controllers`, `dynamixel-workbench-operators` and `dynamixel_workbench_msgs`. We will build these packages using the catkin utility. We assume that ROS has already been installed, for instruction see [here](#ros-installation)

1. Create a separate workspace for workbench installation. We created a separate directory dynamixel_ws
```
> mkdir -p dynamixel_ws/src
> cd dynamixel_ws/src
> git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
> ls 
dynamixel-workbench
```
2. Lets view the package dependecies
```
> cd dynamixel-workbench
# find all files named package.xml in the metapackage, for each package.xml print all lines containing depends keyword.
# this will identify dependencies for each package across all packages in the metapackage
> find -P -name package.xml| while read line; do echo $line; cat $line | grep depend; done
./dynamixel_workbench_single_manager_gui/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>qtbase5-dev</depend>
  <depend>qt5-qmake</depend>
  <depend>dynamixel_workbench_msgs</depend>
  <depend>dynamixel_workbench_toolbox</depend>
  <exec_depend>libqt5-core</exec_depend>
  <exec_depend>libqt5-gui</exec_depend>
./dynamixel_workbench_operators/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>dynamixel_workbench_msgs</depend>
./dynamixel_workbench/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>dynamixel_workbench_controllers</exec_depend>
  <exec_depend>dynamixel_workbench_operators</exec_depend>
  <exec_depend>dynamixel_workbench_single_manager</exec_depend>
  <exec_depend>dynamixel_workbench_single_manager_gui</exec_depend>
  <exec_depend>dynamixel_workbench_toolbox</exec_depend>
./dynamixel_workbench_controllers/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>dynamixel_workbench_msgs</depend>
  <depend>dynamixel_workbench_toolbox</depend>
./dynamixel_workbench_toolbox/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>dynamixel_sdk</depend>
./dynamixel_workbench_single_manager/package.xml
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>dynamixel_workbench_msgs</depend>
  <depend>dynamixel_workbench_toolbox</depend>
```
One can see that `/dynamixel_workbench_single_manager_gui` has additional dependencies. This package is for GUI based dynamixel control, which we don't need. Lets first eliminate this package, and bring out the rest of the packages into the source directory
```
> rm -rf dynamixel_workbench dynamixel_workbench_single_manager_gui
> mv dynamixel_workbench* ../ 
> cd ../
> rm -rf dynamixel_workbench
```
Also in m yinstallation, I found that `sensor_msgs` was not installed. It may not be the case for you if you installed a desktop version of ROS or installed the package previously. We will add sensor_msgs to our existing base installation of ROS. 

3a. Install sensor_msgs on Raspberry Pi: Navigate to ros_catkin_ws and install the sensor_msgs package. This is following guidelines provided [here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)
```
> cd ~/ros_catkin_ws
> rosinstall_generator sensor_msgs --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall
> wstool merge -t src kinetic-custom_ros.rosinstall
> wstool update -t src/
> rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:stretch
> sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
> source devel_isolated/setup.bash
> rospack find sensor_msgs
/home/pi/ros_catkin_ws/src/common_msgs/sensor_msgs
```
3b. Install sensor_msgs on Ubuntu: 
```
> sudo apt-get install ros-kinetic-sensor-msgs
```
4. Build and install dynamixel_workbench
```
> catkin_make
# build complete
> source devel/setup.sh               # add package specific environment variables 
```
#### Testing Dynamixel Workbench installation on Raspberry Pi
The system should be setup such that the Raspberry Pi is connceted to the dynamixel servo through the USB to UART interface. We performed these tests by ssh-ing into Raspberry Pi. 

1. Start the ROS master node
```
> roscore
```
2. Launch the velocity controller node using the provided launch file in the dynamixel_workbench metapackage. This is advised, as the launch contains some parameters relevant to running the velocity controller. You could run this node from commandline but it will be cumbersome to provide all the parameters
```
> roslaunch dynamixel_workbench_controllers velocity_control.launch
```
3. Run the wheel operator
```
> rosrun dynamixel_workbench_operators wheel_operator
[ INFO] [1543808534.781284905]: Set angular velocity(+-0.2 rad/sec) to your Dynamixel!! by using keyboard
[ INFO] [1543808534.781477040]: w : Forward
[ INFO] [1543808534.781530946]: x : Backward
[ INFO] [1543808534.781580530]: a : Left
[ INFO] [1543808534.781630582]: d : Right
[ INFO] [1543808534.781679852]: s : STOPS
```
Now use the keyboard controls move the motor. Ideally, the servos should be mounted onto a chassis. When keyboard controls are sent, one expects the chassis to move in forward or reverse. 
#### Testing Distributed Operation of Dynamixel Workbench 
You could also run the wheel operator node on PC and velocity controller node Raspberry Pi. Here is how you do it. You will run the ros master on PC, wheel operator on PC and velocity controller on Raspberry Pi. 
1. Run ros master on PC and [configure](#configuring-ros-for-pc-and-raspberrypi-3b) for communication with raspberry Pi
```
> roscore
> export ROS_MASTER_URI=http://pc_ip:11311
```
2. Configure Raspberry Pi for communication and run velocity_controller
```
> export ROS_MASTER_URI=http://pc_ip:11311
> export ROS_IP=http://raspi_ip:11311
> roslaunch dynamixel_workbench_controllers velocity_controller.launch
```
3. Now run wheel_operator on PC and control it using keyboard
```
> rosrun dynamixel_workbench_operators wheel_operator
[ INFO] [1543808534.781284905]: Set angular velocity(+-0.2 rad/sec) to your Dynamixel!! by using keyboard
[ INFO] [1543808534.781477040]: w : Forward
[ INFO] [1543808534.781530946]: x : Backward
[ INFO] [1543808534.781580530]: a : Left
[ INFO] [1543808534.781630582]: d : Right
[ INFO] [1543808534.781679852]: s : STOPS
```

## RPLIDAR Setup
RPLIDAR-A1 is a low-cost LIDAR (~$100) from SLAMTec. It has a nominal range from 0.5 - 12m and 360 degree view. It can do 10 rotations per second and collect upto 4000 samples per second. It is being used for indoor mapping applications and also support ROS. 

Here, I will detail installation of RPLIDAR SDK and ROS package. One could use SDK to develop custom ROS nodes for RPLIDAR. 

### RPLIDAR SDK Installation

I will cover this later. 


### RPLIDAR ROS Installation on Raspberry Pi
Here, we detail installation of RPLIDAR ROS packages on Raspberry Pi. I don't need an Ubuntu installation for now as I do not anticipate the need to run any RPLIDAR ROS nodes from PC. In anycase the procedure is pretty much the same. The `rplidar` [package](http://wiki.ros.org/rplidar) provides basic device handling for 2D Laser scanner, RPLIDAR A1. The driver publishes device-dependent sensor_msgs/LaserScan data as required by the [navigation stack](http://wiki.ros.org/navigation/Tutorials/RobotSetup#Sensor_Information_.28sensor_sources.29) 

### Installation on Raspberry Pi
1. Download the package source code from the git repository 
```
> mkdir -p ~/rplidar_ws/src
> cd ~/rplidar_ws/src 
> git clone https://github.com/Slamtec/rplidar_ros
> cd ../
```
We checked the dependencies by inspecting the package.xml file, and found them to be present on the system. One can check if a ros package dependency is present by using `rospack find dep`.

2. Build package using catkin_make
```
> catkin_make
```
3. Source the path
```
> source devel/setup.bash
```
### Testing RPLIDAR ROS package on Raspberry Pi
To verify that one can acquire data from RPLIDAR running on Raspberry Pi and view it on PC, take the following steps.
1. Configure PC for viewing laser scan data: We will run ros master on PC and then run rviz. Rviz will be configured to listen to sensor_msgs/LaserScan topic type. 
On PC, configure ROS_MASTER
```
> export ROS_MASTER_URI=http://192.xxx.xxx.xxx:11311
```
Then start roscore
```
> roscore
```
Also start rviz
```
> rosrun rviz rviz
```
We'll come back to configure rviz later

2. Configure and run `rpLidarNode` on Raspberry Pi. 
On Raspberry Pi, configure ROS_MASTER and ROS_IP
```
> export ROS_MASTER_URI=http://192.xxx.xxx.xxx:11311
> export ROS_IP=http://192.xxx.xxx.xxx
```
Launch rplidar.launch file. This will start the `rplidarNode` on Raspberry Pi, with some present parameters in the launch file
```
> roslaunch rplidar_ros rplidar.launch
```
3. Configure rviz to view laser scan data
On the PC, we can verify that rplidarNode is running, by invoking
```
> rosnode list
/rosout
/rplidarNode
```
You should also be able to see the rostopic `/scan` being published by rplidarNode. We need to capture this topic in rviz to view the live scan data. 


In rviz, add the topic scan in the Displays list. It will however throw the following error
```
For frame [laser]: Fixed Frame [map] does not exist
```
This is because we have not setup a tf tree that relates the /map frame and the /laser frame 
We need to change the global settings in rviz. Set the `Fixed Frame` to `/laser`. Now you should be able to view the scan data as shown in the image below


![rplidar rviz](images/rplidar_rviz.png)


Also change the view setting Type to TopDownOrthogonal as show below


![topdown_ortho](images/topdown_ortho.png)



The scan should look something like this


![rplidar scan rviz](images/rplidar_scan_rviz.png)

5. From here, one can use the /scan data for mapping or navigation purposes. 

The details for configuring `rplidarNode` parameters are not covered. We will cover them at a later stage as needed


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
- [Dynamixel Protocol 2.0](http://emanual.robotis.com/docs/en/dxl/protocol2/)
- [RPiLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)
- [RpiLIDAR User Manual](docs/slamtec_lidar_user_man.pdf)
- [RpiLIDAR Protocol](docs/slamtec_lidar_interface_protocol.pdf)


ROBOTIS Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The Dynamixel API requires programming in C/C++. We wull use the recommended Protocol 2.0 for communicating with the X-series servo. We will test the SDK on Ubuntu 16.04 LTS running on Intel X86 and Raspbian running on Raspberry Pi 3. We will use FT232RL serial board from Sparkfun with custom circuit to perform half-duplex communication with the servo. Finally we will also test the dynamixel using ROS C++ package on Ubunutu or Raspbian. 

# Hardware Setup

