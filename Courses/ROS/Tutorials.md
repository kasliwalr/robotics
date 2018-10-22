# ROS Tutorials


## Installation
There is more than one distribution of ROS supported at a time. Some are older releases with long term support, other are newer releases with shorter support lifetimes.

The following versions are recommended
1. [ROS Kinetic Kame](http://www.ros.org/reps/rep-0003.html#kinetic-kame-may-2016-may-2021) - supported until April, 2021
2. [ROS Melodic Morenia](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023) - supported until May, 2023



### Which Distribution to Use?
Based on the [recommendation](http://wiki.ros.org/Distributions).

|New Capability|Major Update Frequency|Recommended Distro|
|--------------|----------------------|------------------|
|Preferred but not required|Not preferred|Latest LTS|
|Much Preferred|Acceptable|Latest|
|Much Preferred|Not Preferred|Switch to Latest LTS every 2 years|
|Specific Platform Required (other than Ubuntu 16.04| See REP-3|
|Newer Gazebo needed||Kinetic for Gazebo v7|
|Use opencv3||Indigo or later|

 
### ROS Kinetic Kame Installation Instructions for Ubuntu 16.04 LTS

We will not build the ROS using source code, rather we'll use the recommended method of installing ROS using debian packages

1. Set up source list
2. Obtain key from ROS key server using `apt-key`
```
> sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
[sudo] password for rk: 
Executing: /tmp/tmp.9JOTvWUECn/gpg.1.sh --keyserver
hkp://ha.pool.sks-keyservers.net:80
--recv-key
421C365BD9FF1F717815A3895523BAEEB01FA116
gpg: requesting key B01FA116 from hkp server ha.pool.sks-keyservers.net
gpg: key B01FA116: public key "ROS Builder <rosbuild@ros.org>" imported
gpg: Total number processed: 1
gpg:               imported: 1

```
3. update the debian package index 
```
> sudo apt-get update
```
4. We will use the full-desktop installation of kinetic-kame
```
> sudo apt-get install ros-kinetic-desktop-full
```
This installation will take about 450MB of space. 

5. Configure rosdep: rosdep is a command-line tool for installing system dependencies. For *end-users*, rosdep helps you install system dependencies for software that you are building from source. [Here](http://docs.ros.org/independent/api/rosdep/html/commands.html) are usage instructions

```
> rosdep init
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

	rosdep update
> rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index.yaml
Add distro "groovy"
Add distro "hydro"
Add distro "indigo"
Add distro "jade"
Add distro "kinetic"
Add distro "lunar"
Add distro "melodic"
updated cache in /home/rk/.ros/rosdep/sources.cache
```

6. Configure bashrc to load ROS environment variables automatically


### Maintaining ROS Environment
ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier. 

To check if ROS environment is set properly, print your environment variables and look for ROS environment variables 
```
> printenv | grep ROS
```
If they are not, then some setup.sh files may need to be sourced, or hardcoded into into .bashrc. 
Environment setup files are generated 3 ways
1. ROS packages installed with package managers provide setup.sh files
2. rosbuild workspaces provide setup.sh files using tools like rosws
3. setup.sh files are created as a byproduct of building or installing catkin packages



### Creating a ROS Workspace
This pertain to catkin workspaces. We'll come back to this later



# Catkin

## Overview
catkin is the official build system for ROS. catkin combines CMake macros and python scripts to provide functionality on top of CMake's normal workflow. 


Catkin is a build system, in context of ROS, source code is organized into packages. After building the packages, it will contain many targets, which were build using the build system. 

ROS utilizes catkin as a custom build system which extends CMake to manage dependencies between packages. 

## Why do we need catkin for ROS?

