# Indoor Mapper
This project details the design of an indoor mapper project. This is an ongoing project, and will add capabilities over time


# Requirements 

Features, Form Factor, Look, Cost etc
## Overview

### Objectives
Why are we doing this project?
- survey project for tools, techniques and algorithms used in robotic mapping and navigation - ROS, slam algorithms, lidar sensors, imu sensors, sensor fusion and some code development in C++
- solving a useful problem of indoor navigation
- how to test slam systems
- employ embedded hardware for robotics application - microcontrollers, sbcs and gpus
- embedded OS

### Process
- we will be using low cost navigation platform turtlebot3, and customize it for our application needs and cost constraints
-
### Roles and Responsibilites
- this project has been solely developed by me. I am the designer, developer and tester. 
### Interactions with Existing Systems
- the hardware used will be a standard PC running Ubuntu 16.04 LTS
- hardware will be Raspberry Pi running Raspbian Stretch with suitable additional packages
- ROS will be used for performing computational, sensing and actuation tasks
- Any custom ROS nodes will be written in C++ using the roscpp API
- ROS navigation and mapping stack/packages will be used for mapping and navigation tasks
- wherever possible already available ros packages will be used for hardware interfacing
- navigation and mapping will be completed using lidar sensing, imu and odometry sensors built into the servos
- raspberry Pi 3B will be used interfacing with sensors
- raspberry Pi and PC will interface over the Wifi
- the robotic platform will be built using turtlebot3 plates from robotis
- the servos for platform will be dynamixel x430-w250T. 
- any 3S lipo battery that meets with suitable C rating and sufficient juice will be used. It must fall with dimensions of 
- there will be a android mobile app fro remotely controlling the robot and receving cached information from it. 
- the data engine will upload information on a database server regarding the robot




### Terminology
**mobile_base_link**
**lidar_link**
**imu_link**


### Security
All software and hardware related files will be available either as pdf or plaintext files from github.  
Any operating systems, ROS framework, third party drivers are downloadble from respective websites. 
Therefore we do not anticipate spreading of any virus.
All components whether software or hardware used in this project are open source. The project is open source as well. Any custom nodes written for this project are open source. 



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

## Functional Description
### Functionality
- the robot shall be capable of moving on plane ground (with pot holes of size < 5mm)
- it will perform mapping of an indoor invironment spread over <= 1000 sq ft
- it will perform navigation of an indoor environemnt spread over <= 1000 sq ft, in < 30 min
- user shall be capable of interrupting map building/navigation abruptly and take control of the robot for remote navigation
- the user shall be able to set navigation and mapping parameters remotely. 
- the robot shall be able to navigate and mapping among objects found in a typical home such as a sofa (of various materials), wooden furniture, books, paper, glass doors, plastics, metal etc.
- the robot shall have a button to control power to the system
- the user shall be able to specify different modes of operation - low power and high power. 
- the system shall estimate the amount of battery consumption report that to the system intelligence
 
### Scope
Phase1: specify the functional requirements
Phase2: system specifiction. Hardware and software modules of the system will be defined
Phase3: Different ROS nodes for each subsystem will be developed and tested. We will also demonstrate that they meet specifications
Phase4: System integration and testing will be performed. We will demonstrate that the system meets performance and functional specifications

### Prototypes
- We will demonstrate the functioning of the individual modules using small prototypes. 
- We will also demonstrate the functioning of the overall system using a full system prototype. This will demonstrate a simple autonomous mapping and navigation tasks and remote communication and control
- We will demonstrate a fully functional robot with complex mapping and navigation tasks and remote control and communication

### Performance
- The robot shall be able to perform mapping of a 1000 sq ft area. After completion of mapping, it should be able to return to the start location.  It should complete the mapping in < 1hr and on a single battery charge. 
- It should be able to perform navigation of area < 1000sq ft, within 30 min and on single battery charge
- It should weight less than 3 pounds. 

### Usability
This is how the user interface will look like
1. From the PC: 
- command line interface to set the robot mode - autonomous mapping, autonomous exploration, remote control
- in autonomous mapping, the user will have the option of viewing the map as it is being created/ or not
- in autonomous navigation, the user will have the option of viewing the overlay of the explored map and overall map/ or not
- in remote control, the user will have keyboard control to 
  - move the robot fwd/back, left, right. 
  - stop/start the lidar. 
  - do finite lidar capture. 
  - do imu capture. 
  - shutdown power to imu/lidar/servos
  - reboot individual ROS nodes
  - purge temp data on board robot and other housekeeping tasks
- in remote control, the user shall have GUI control
  - to specify goal for the robot to navigate to. 

2. From the mobile App:
GUI interface to set the robot mode - autonomous navigation and remote control
- in autonomous navigation, the user will have the option of viewing the overlay of the explored map and overall map
- in remote control, user will have keyboard control to
  - reboot individual ROS nodes
  - purge temp data on board robot and other housekeeping task

### Safety

## Deliverables
### Reports
A requirements document. 
Code files
Hardware Design documents
Assembly / Setup Instructions
Software code (with comments), Flowcharts, Callgraphs, Dataflow graphs, Algorithm Descriptions
Test Report: setups and results
Demo Video

### Code
### Video
