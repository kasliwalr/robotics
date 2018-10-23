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
This pertain to catkin workspaces. We'll come back to this later. For now we'll simply create a **catkin workspace** and move on

Create a directory anywhere you want, preferrably somewhere in user's home directory on linux, name it anyway you want, we choose an apt name `catkin_ws`
```
> mkdir catkin_ws
> cd catkin_ws; mkdir src
> catkin_make
```
catkin_make is one of many catkin commands. It generate a `CMakeList.txt` in the src directory. In addition, it will create a `devel` development directory, and an `build` installation directory. 
The devel directory contains setup.sh files, which contains redefinition of some ROS environment variables. Sourcing these files will overlay your workspace on top of the ROS environment. 
We want to work in this workspace, so we'll source the setup files
```
> source devel/setup.sh
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.

```
> echo $ROS_PACKAGE_PATH
/home/rk/repos/Robotics/scratch/catkin_ws/src:/opt/ros/kinetic/share
```
## Navigating the ROS Filesystem

### Finding package location
In ROS, code is spread over many packages. We'll cover a few role command-line tools used to navigate the ROS filesystem. To do these exercise, we'll first install a tutorial package from the online ROS debian package repository using apt-get.
```
> sudo apt-get install ros-kinetic-ros-tutorials
```
We'll use command-line tools `rospack` to find packages. 
```
> rospack find roscpp_tutorials
/opt/ros/kinetic/share/roscpp_tutorials
```
The above show the location where roscpp_tutorials are installed by apt-get. 

### Navigating to package location
We use roscd for it. We only need to provide the package name to the command
```
> pwd
/home/rk/repos/Robotics/scratch/catkin_ws
> roscd roscpp_tutorials
> pwd
/opt/ros/kinetic/share/roscpp_tutorials
```
roscd can also move to a subdirectory of a package like so
```
> roscd roscpp_tutorials/srv
> pwd
/opt/ros/kinetic/share/roscpp_tutorials/srv
```
`roscd log` will take you to the directory where ROS log files are stored. 

### Listing directory contents
`rosls` allows you to `ls` directory contents simply by refereing to the package name. Here is an example
```
> rosls roscpp_tutorials
cmake  launch  package.xml  srv
```

## Creating a ROS package
### Creating a package using catkin
Since we will use the catkin build system to manage this package, its of type *catking package*. By creating, we mean we'll add some content to it. There is some boilerplate stuff that needs to be in the package folder, for it to be called a package. 
1. The package must contain a catkin compliant **package.xml**
2. The package must contain a **CMakeLists.txt** which uses catkin
3. Each package must have its own directory, this means no nested packages, or multiple packages sharing the same directory. 


The recommended method of working with catkin packages is using a catkin workspace, but you can also build catkin packages standalone. We will demonstrate catkin package creation using `catkin_create_pkg` script. 

catkin packages will be created in **src** directory of catkin workspace
```
> cd catkin_ws/src
> catkin_create_pkg package_name  dep1 dep2.... depn                        # dep1... depn are an optional list of dependencies on which the package depends
> ls
beginner_tutorials  CMakeLists.txt
```
As you can see a new directory of name package_name has been added. This has additional contents. At the minimum it will contains CMakeLists.txt and package.xml file. 

### Building a catkin workspace
Navigate back to the top level folder, and invoke catkin_make
```
> cd ../catkin_ws
> pwd
catkin_ws
> catkin_make
> ls

```
After the workspace has been built, it will have a structure similar to /opt/ros/kinetic. In a sense it is expected, because ROS distribution is a collection of packages and so is a workspace. 
To add workspace to ROS environment, it needs to be sourced
```
> source devel/setup.sh
```

### Dependency Management
rospack can be used to inspect the first order dependencies of a package like so
```
> rospack depends1 beginner_tutorials
roscpp
rospy
std_msgs
```
In many cases, a dependency will also have its own dependency, one can use rospack pack in a similar manner, just be replacing the package name with a new package's name
```
> rospack depends1 rospy
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```

To list all dependencies for a package (direct and indirect) recursively, do like so
```
> rospack depends beginner_tutorials
```

### Customizing the package
For this we'll need to delve deeper into the structure of a package and even the files therein. 

### File Package.xml
There is one such file per package, it should be located in `src/package_name` directory correponding to the package *package_name*. 
[see detailed description](#package-xml-structure)


## Building a ROS package
If all system dependencies are installed, a package is ready to be built
[TODO: continue here](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)









# Catkin

## Overview
catkin is the official build system for ROS. catkin combines CMake macros and python scripts to provide functionality on top of CMake's normal workflow. 


Catkin is a build system. In context of ROS, source code is organized into packages; After building the packages, it will contain many targets, which were build using the build system. 

ROS utilizes catkin as a custom build system which extends CMake to manage dependencies between packages. 

## Why does ROS have a custom build system?
In short conventional tools like `Make`, `CMake` etc are difficult to use on larger projects like ROS. In additional ROS is a collected of larger number of heretergenous packages, which sometimes have their own directory structure and slightly different build rules. catkin tries to make it easier to build and share ROS packages.

## Catkin Workspace
catkin packages can be built as standalone projects, in the same way cmake projects can be built. Catkin however, provides support for workspaces, where it can build multiple, interdependent packages together all at once. 

### What is a catkin workspace?
It is a directory where you can modidy, build and install catkin packages. The typical layout of catkin is specified [here](http://wiki.ros.org/catkin/workspaces)
It has 4 directories at the topmost level - **src, devel, build and install**. These 4 directories serve different purposes in the software development process. Here is a brief description
1. **src**: contains source code of all the catkin packages. This is where you clone/checkout source code for packages you want to build. Each directory within source space contains one or more catkin packages. The root of the source space contains a symbolic link to catkin's toplevel CMakeLists.txt file. The file is invoked by CMake during configuration of the catkin projects in the workspace. This link is generated by invoking `catkin_init_workspace` command
2. **build**: This is where CMake is invoked to build catkin packages (which are located in source space). CMake and catkin keep their cache information and other intermediate files in this space. 
3. **devel**: Built targets are placed here prior to being installed. The way targets are organized in devel space is same as their layout when they are installed. This space provides a useful testing and development environment which does not require invoking the installation step. The location of development space is controlled by catkin specific CMake variable CATKIN_DEVEL_PREFIX. Although its default value is <build_space>/devel_space, it is recommended to set devel space to be a peer of build space
4. **install**: Once targets are built, they can be installed into the install space. Since the install space is set by the CMAKE_INSTALL_PREFIX, it defaults to `/usr/local`. 


### package.xml structure
The package manifest is an XML file called package.xml that must be included with any catkin-compliant package's root folder. This file defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages.

package.xml uses format2 which is the new and recommended one. For more details see [here](http://docs.ros.org/indigo/api/catkin/html/howto/format2/index.html)

Here, we briefly cover it

1. Each package.xml file has the <package> tag as the root tag in the document.
```
<package format="2">
 
</package>
```
2. Required tags: These minimal set of tags that need to nested inside `<package>` tag. 
- <name>: package name
- <version>: 3 dot-separated version number of package
- <description>: description of package contents
- <maintainer>: name of person maintaining the package
- <license>: software licenses under which the code is released

Here is an example for `beginner_tutorials/package.xml`
```
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>
  <maintainer email="rk@todo.todo">rk</maintainer>
  <license>TODO</license>
</package>
```

3. Dependencies: The file also contains tags to identify the package dependencies, it can have 6 types of dependencies

|Dependency|Description|Tag|
|----------|-----------|---|
|**build**|packages needed to build this package. This include compile time, link time etc. In cross-compiling scenario build dependencies are for the targeted architecture|<build_depend>|
|**build export**|packages needed to build libraries against this package|<build_export_depend>|
|**execution**|packages needed to run code in this package. This is the case when you depend on shared libaries|<exec_depend>|
|**test**|additional dependencies for unit tests. Never duplicate any dependencies already mentioned as build or run dependencies|<test_depend>|
|**build tool**|build system tools needed by the package to build itself. In cross-compilation scenarion, build tool dependencies are for the architecture on which the compilation is performed.|<buildtool_depend>|
|**documentation tool**|documentation tools which package needs to generate documentation|<doc_depend>|

<depend> specifies that a dependency is a build, export, and execution dependency. This is the most commonly used dependency tag.


4. Metapackages: It is convenient to group multiple packages as a single logical package. This can be accomplished through **metapackages**. metapackages have an additional tag, <export> 
```
<export>
 <metapackage />
</export>
```
Other than the build tool dependency on catkin, metapackages can only have execution dependenices on packages which they group. Additionally, metapackage has a required boiler-plate CMakeLists.txt  file
```
cmake_minimum_required(VERSION 2.8.3)
project(<PACKAGE_NAME>)
find_package(catkin REQUIRED)
catkin_metapackage()
```
5. Additional tags: Additional tags inlcude the <url> and <author> tags. These are self-explanatory. 

