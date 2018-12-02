# Indoor Mapper Requirements
This project details the design of an indoor mapper project. This is an ongoing project, and will add capabilities over time
Here we detail the requirements for IndoorMapper robot project. This will be a changing set of requirements as we explore the project through initial prototyping. We also expect these requirements to be fine tuned in the later stages of the project. The following sections are organized as follows

- Overview section 
  - [Objectives](#objectives)
  - [Engineering process](#engineering-process)
  - [Roles and responsibilities of project team](#roles-and-responsibilities)
  - [Interaction with existing hardware and software technologies](#interactions-with-existing-systems)
  - [Security](#security)
- Functional Description
  - [Functional requirements](#functional-requirements)
  - [Performance requirements](#performance-requirements)
  - [Usability requirements](#usabilitiy-requirements)
  - [Extensibility](#extensibility)
  - [Scope](#scope)
  - [Prototypes](#prototypes)
- [Deliverables](#deliverables)

## Overview
### Objectives

Let's define the project scope. The project will build a wheeled robot capable of moving on plane surfaces in an indoor environment. It will have necessarily have the sensor array for mapping and navigation (lidar or some other range finder, odometry - wheel encoder/imu/gps). The first task that the robot shall accomplish is building a map of an indoor environment (office/home etc) and then navigate to a particular spot in the map when directed to do so. In parallel, we will add the functionality to recognize common household objects such as glass, salt shaker, phone, charger etc.
This will necessarily need a camera for taking picture and doing inference on whether it is of a certain type. When both these functionalities are complete, we want the robot to be able to search for an object in indoor environments and then report the object's location. 

One the human interface end, the robot shall be able to recognize human commands such as "Follow me/Come here/Stop/Start/Speed Up/Speed Down etc". Or it could recognize hand gestures. There is also the possibility that we will mount the robot with an RGB-D camera for depth sensing applications. 

Finally, we want the robot to perform actuation tasks. We want to equip the robot to be able to pick objects of certain size and shape. This will possibly be another project where we build and perfect a robotic arm. We hope we could deploy this robotic arm on the robot, so that the robot could perform a sample search and return in an indoor environment. 

During this whole process, we want to explore algorithms/strategies to continually improve our performance. Specifically, we want to explore RL techniques for navigation.

The formal requirements for the first phase of the project are defined [here](Requirements.md). The first phase is developing a wheeled robot capable of autonomously navigating an indoor environment. Search and Object Grabbing is not part of its responsibilities. 



I am doing this project to learn and apply tools and techniques used for autonomous mapping and navigation applications in robotics. This is a starter project with enough complexity to invoke use of advanced mapping and navigation algorithms while keeping the objectives simple. Some specific tools and techniques that I am interested in deploying in this project 
- Robotic Operating System (ROS) framework
- LIDAR sensing
- IMU sensing
- SLAM algorithms
- sensor fusion for odometry
- ROS Navigation Stack. 


Besides the mapping specific applications, I hope to explore use of single board computers, microcontrollers, GPUs and software stacks associated with those. 

### Engineering Process
I am taking a fast prototyping approach by using existing hobby kits. Specifically, I will use the [Turtlebot3 Burger](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) platform from [Robotis Inc.](http://robotis.us/). I will however, not use the out of the box version, instead will only use some Robotis hardware (frame, wheels, dynamixel servos etc.) along with custom made parts (primarily to reduce costs). I will choose the robot computers and set the software configuration for accomplishing the navigation and mapping task. Software development will be done using C++, Python and ROS framework. 

I will also explore the distributed paradigm for our robot. The intelligence for the robot will be onboard a PC, while drivers will be onboard the robot computer. The PC will use a Linux based system. 

To speed up application development, I will use off-the-shelf sensors and actuators, with well developed drivers. My main focus will be on tuning and confuring the algorithms and getting better performance from the system. I will also develop tests for any custom written code as well as for modules developed using ROS framework. 

### Roles and Responsibilites
I am the owner of this project. In this role, I am the architect, developer and tester and project manager. However I view the responsibilities primarily that of a system developer and tester since much of the electronics and mechanical design is open-source. 

### Interactions with Existing Systems
The IndoorMapper robot hardware primarily resides on the mobile robot build around the Turtlebot3 Burger platform. The software resides both on the Laptop running Ubuntu 16.04 LTS and robot computer which is a Raspberry Pi3 running Raspbian Stretch. I am using ROS framework to handle communication between different robotic tasks and will use either already available nodes (in ROS packges) or custom written C++ ROS packages written using C/C++ driver libraries. For automation of tasks both on the PC and Raspeberry Pi, I may use shell based scripts (bash/ksh) and Linux system calls. 

I have used some standard or widely used sensors used for mapping and navigation. The choice of sensor types for mapping and navigation depends on the algorihtms used. In our case I used a LIDAR sensor, IMU sensor, and encoders deployed in dynamixel servos. I have not applied any thought in choice of servos except that I will use one - I have settled with the default one that comes with Turtlebot3 Burger. 

To keep robot light and get sufficiently high steady state current, I will use a LiPO battery and any associated electronics for their safe operation. 

The user interface for the IndoorMapper robot will either consist of a mobile Android App or a webpage interface. I expect the need for some kind of a webserver running on a machine that is accessible over both WLAN and internet. 

### Terminology
**mobile_base_link**
**lidar_link**
**imu_link**


### Security
The IndoorMapper robot initial version is not intelligent enough to recognize and take evasive action to threats, so I will not slow ourselves myself with these concerns. I assume it will be able to roam in a navigable region. 

The security aspect that we will focus on is the remote control of the robot. In our scheme, another PC can send and receive data to the robot, therefore we do not want an unauthorized machine to have access to data by the robot, not have ability to send spurious data to the robot. Also, since ROS nodes running on PC/robot may provide services, we cannot allow them to be blocked by any DoS attacks. Since many nodes run on the PC which is used for other applications, we also need to ensure that IndoorMapper components on PC running in a protected environment no accessible to other user apps and even other users on the system. 

All software and hardware related files will be available either as pdf or plaintext files from github.  Any operating systems, ROS framework, third party drivers are downloadble from respective websites. Therefore we do not anticipate spreading of any virus. All components whether software or hardware used in this project are open source. The project is open source as well. Any custom nodes written for this project are open source. 


## Functional Description
### Functionality
- the robot shall be capable of moving on plane ground (with pot holes of size < 5mm)
- it will perform mapping of an indoor environment spread over <= 1000 sq ft
- it will perform navigation of an indoor environment spread over <= 1000 sq ft, in < 30 min
- user shall be capable of interrupting map building/navigation aand take control of the robot for remote navigation
- the user shall be able to set navigation and mapping parameters remotely. 
- the robot shall be able to navigate and mapping among objects found in a typical home such as a sofa (of various materials), wooden furniture, books, paper, glass doors, plastics, metal etc.
- the robot shall have a button to allow the operator to control power to the system
- the user shall be able to specify different modes of operation - autonomous mapping, autonomous navigation, remote control
- the system shall estimate the amount of battery consumed 

### Performance
- The robot shall be able to perform mapping of a 1000 sq ft area. After completion of mapping, it should be able to return to the start location.  It should complete the mapping in < 1hr and on a single battery charge. 
- robot shall be able to perform navigation of area < 1000sq ft, within 30 min and on single battery charge
- robot shall weoght less than 3 pounds. 

### Usability
This is how the user interface will look like
1. From the PC: 
- command line interface
  - set the robot mode - autonomous mapping, autonomous exploration, remote control
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
  - specify goal for the robot to navigate to. 

2. From the mobile App:
- GUI interface 
  - set robot mode - autonomous mapping, autonomous exploration
  - in autonomous navigation, the user will have the option of viewing the overlay of the explored map and overall map
  - in remote control, user will have keyboard control to
  - reboot individual ROS nodes
  - purge temp data on board robot and other housekeeping task

### Extensibility
I will not add much here except that the robot should be able to support additional robot computer and sensors. The platform provided by Robotis Inc. is capable of supporting this goal as [evidenced](http://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#locomotion) by a variety of robot architecture they provide. 

### Scope
- Phase1: specify the functional requirements
- Phase2: system specifiction. Hardware and software modules of the system will be defined
- Phase3: Different ROS nodes for each subsystem will be developed and tested. We will also demonstrate that they meet specifications
- Phase4: System integration and testing will be performed. We will demonstrate that the system meets performance and functional specifications

### Prototypes
- I will demonstrate the functioning of the invidual subsystems or an essential component of their functionilty sufficient to show that overall subsystem will work
- I will demonstrate the functioning of the IndoorMapper robot using a full system prototype. This will demonstrate a simple autonomous mapping and navigation task and remote communication and control
- I will demonstrate a fully functional robot with complex mapping and navigation tasks and remote control and communication

## Deliverables
Below is a list of deliverables for the project
- System Requirements Document
- Assembly and Setup Manual
- Source code (accessible via github)
- Test Report
- Electrical Schematics
- Software Documentation - flow charts, callgraphs, algorithm descriptions etc
- Demo Video
