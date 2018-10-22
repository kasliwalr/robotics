[ROS Overview](#ros-overview)

[ROS Course](#ros-course)


# ROS Overview

## What is ROS?
It is a meta-OS for a robot. Like an OS, it provides certain services, abstracts the hardware and facilitates message passing between processes. In addition, it also provides package management. It also provides tools for downloading, building, writing and running code across multiple computers. 

ROS runtime can be described as a peer-to-peer network of running processes that communicate using the ROS infrastructure. ROS implements synchronous RPC style communication over *services* and asynchronous communication style using *topics*. 

## What were the goals behind creation of ROS?
Primary goal is to support code reuse. As mentioned above, ROS runtime is a peer-to-peer network of running processes. Each processes is a running executable. These executables can be individually designed and maintained. ROS enables these executables grouped into *Packages* and *Stacks* and distributed for reuse. 

In addition to this main goal, there are other desirables that ROS tries to meet

1. Thin: ROS is designed to be as thin as possible
2. ROS-agnostic libraries: Libraries should be ROS agnostic with clean functional interfaces
3. Lanugage Independence: ROS is not heavily language dependent. There are implementation of ROS in python and C++

## ROS Concepts
ROS has three levels of concepts - **filesystem** level, **computation graph** level and **community** level. ROS has a naming scheme, wherein it uses two types of names - **Package Resource Name** and **Graph Resource Name**

### File System Level
At file system level, we deal with ROS resources found on the disk

1. **Packages**: Main unit for organizing code. It is the most atomic build and release item in ROS. It may contain a ROS runtime processes' executable, ROS-dependent library, datasets, configuration files etc. 
2. **Metapackages**: Specialized packages that serve to represent a group of related packages. 
3. **Package Manifests**: Manifests are xml files, which contain metadata about a package such as its name, version, description, license information, dependencies etc
4. **Repositories**: A collection of packages that share a common version control system. Packages that share a VCS, share the version number, and can be released together. Repos can also contain just a single package
5. **Message types**: Message descriptions. These are stored in package/msg/MessageType.msg files. They define the data structure for **messages** send in ROS
6. **Service types**: Service Description. These are stored in package/srv/ServiceType.srv. They define the **request** and **response** data structure for services in ROS. 

### Computation Graph
ROS runtime is a peer-to-peer network of ROS processes. The basic concepts in this graph are as follows:
Below is a picture of a computation graph, the ovals are nodes in the graph, they represent ROS nodes. The edges represent the ROS message streams. 
![Computation Graph](images/computation_graph.png)


1. **Nodes**: Nodes are processes that are performing computation. A node is a piece of code, that is written using ROS client libraries such as **roscpp** or **rospy**. 
2. **Master**: ROS master provides name registration and lookup to all other nodes in the computational graph. 
3. **Parameter Server**: stores data dictionary, accessible by key in a central location. It is part of the Master. 
4. **Messages**: Nodes communicate with each other by passing messages
5. **Topics**: Messages are routed via a transport system with publish/subscribe semantics. A node sends out a message by publishing it to a topic. The **topic** is a **name** that is used to identify the content of the message. One can think of topic as a strongly typed message bus. 
6. **Services**: Request/Reply is achieved via services which are defined by a pair of message structures. A serving node provides a **service** under a **name**. The client node sends a request to this service and awaits a reply. ROS client API presents this as an RPC cal. 
7. **Bags**: a format for saving and playing back ROS message data. They are an important mechanism for storing data. 

#### How does the computation graph work?
//TODO

### Community
These concepts relate to ROS resources that help the community exchange software

1. **Distributions**: collections of versioned stacks that you can install on your system. Distributions play a similar role to Linux distributions, they make it easier to install a collection of software, and they also maintain consistent versions across a set of software.
2. **Repositories**: ROS relies on a federated network of code repositories
3. **ROS Wiki**: Main forum for documenting information about ROS. Anyone can sign up for an account and contribute their own documentation, write tutorials etc.
4. **Mailing List**: mailing-list to get new updates, and ask questions
5. **ROS Answers**: Q&A site for ROS related questions
6. **Blog**: provides regular updates

### Names

#### Graph Resource Names
These names provide a heirarachichal naming structure that is used for all resources in ROS Computation Graph. So this is a systematic method of assigning names to **nodes**, **parameters**, **topics** and **services**. 

Graph Resource Names provide encapsulation, to expand on it, each resource is defined within a namespace. It can address other resources within its namespace as well as those in namespaces above it. Resources are allowed only to create other resources within their own namespace. If connections need to be made b/w two distinct namespaces, then that needs to facilitated by some code, above both namespaces. 

**Name Resolution**</br>
Names are resolved relatively i.e. if I am a resource, in need of accessing another resources, I will simply look for that name. Implicitly, I will be looking in the namespace I belong to. If not found here, I will be looking in the namespace up one level. 

To understand name resolution better, it is important to understand that there are 4 types of graph resource names - **global**, **relative**, **private** and **base**. 
    
    
 |Name Type| Description|
 |---------|------------|
 |relative |Resolution is done relative to node's namespace, so for /wg/node1, the namespace is /wg, so node2 will resolve to /wg/node2|
 |base     |Names with no namespace qualifiers are base names|
 |global   |Names that start with a / are global. They are considered fully resolved, and no name resolution is performed for them|
 |private  |Names that start with a tilde, ~, are private. When resolving them, the parent node's name is considered a namespace|
 
 
 Here are some examples
 
|Node|Relative (default)|Global|Private|
|----|------------------|------|-------|
|/node1|bar -> /bar|/bar -> /bar|~bar -> /node1/bar|
|/wg/node2|bar -> /wg/bar|/bar -> /bar|~bar -> /wg/node2/bar|
|/wg/node3|foo/bar -> /wg/foo/bar|/foo/bar -> /foo/bar|~foo/bar -> /wg/node3/foo/bar|


**Remapping**</br>
Any name within a ROS node can be remapped. More on this later. 

#### Package Resource Names
These names provide a way of referring to ROS filesystem resources. Typically any resource name is prefixed with a package name. ROS is able to resolve it because it is able to locate Packages on disk and make certain assumptions about the structure of the Package. Let's understand it better through some examples

|Name|Resolution|Description|
|----|----------|-----------|
|std_msgs/String|path_to_std_msgs/std_msgs/msg/String.msg|Messages are always stored in msg subdirectory and have a .msg extension|
|foo/bar|/path_to_foo/bar|searches for an executable file named bar in package foo|

## ROS High-Level Concepts
ROS is pretty general, and is designed in such a way so as to be agnostic about the robot architecture. The ROS makers claim that to be build larger system, there is need for additional generalized higher level concepts. We'll go over these

### Coordinate Frames/Transforms
The `tf` is a ROS package. It provides a distributed framework for calculating poses (position + orientation) of multiple coordinate frames over time. 

### Actions/Tasks
The `actionlib` package defines a topic based interface for preemptible tasks in ROS. The actionlib enables a fire-and-forget interface. Unlike service which is a blocking call, actionlib tasks are non-blocking. Further they are allowed to be preempted if taking too long for example. As far as the use cases go, services are used for quick updates, the request-response cycle is completed very quickly. If the response may take much longer, actionlib tasks are the appropriate choice. 

Here is a [discussion](http://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X)

### Message Ontology
The *common_msgs* stack provides a base message ontology for robotic systems. It defines several classes of messages

1. *actionlib_msgs*
2. *diagnostic_msgs*
3. *geometry_msgs*
4. *nav_msgs*
5. *sensor_msgs*

### Plugins
plugin stack provides an API for dynamically loading C++ libraries. 
//TODO: well revisit this later

### Filters
`filters` packages is a C++ library for implementing processing pipeline using a sequence of filters. The package contains a base class `Filter` upon which specific implementations of filters are built. When there is a need to use more than one filter, a filter chain `filter::FilterChain` object is created.
//TODO: We'll come back to this later

### Robot Model
This refers to the `urdf` packages. This package contains a C++ parser for reading xml files detailing the robot model. This package contains a number of XML specifications for robot models, sensors, scenes, etc. Each XML specification has a corresponding parser in C++. 

## ROS Client Libraries
ROS client library is a collect of code, that maps the ROS concepts to C++ based ROS programming API. Here is a summary of `roscpp` API

|API Name| Description|
|--------|------------|
|ros::init()|A version of ros::init() must be called before using any of the rest of the ROS system|
|ros::NodeHandle|Public interface to topics, services, parameters, etc.|
|ros::master|Contains functions for querying information from the master|
|ros::this_node|Contains functions for querying information about this process' node|
|ros::service|Contains functions for querying information about services|
|ros::param|Contains functions for querying the parameter service without the need for a ros::NodeHandle|
|ros::names|Contains functions for manipulating ROS graph resource names|


## ROS Tools
Now, we'll give an overview of some ROS specific developer tools

### rocore
roscore is the first step in bringing up the ROS system. It starts three different objects namely ros name server `master`, ros `parameter server` and ros node `rosout`. 
1. ros `master`: it is the name server
2. ros `parameter server`: holds key/value parameter data
3. `rosout` node: aggregates debug messages from all other nodes

When a node is started, it first registers itself with the master. The master maintains a lookup table, where each node name maps to its network address. So after registration, the lookup table is updated with an entry containing the node's name and corresponding network address. 

Node also registers in topic subscriptions, topic advertisements and services. It also obtains the network address for all publishers to a topic that it subsrcibes to. It will then contact each publishing node to negotiate a connection to receive messages on that topic. 

If a master is killed, the system state cannot be changed. This means new nodes cannot be added, and new connections can't be made. 


### Parameter Server
The job of parameter server is to store configuration data in a network accessible database. It maintains a dictionary of key/value pairs, where key is a string, and values can be of any type. Any node can read and write to parameter server. 

The `rosparam` tools allows operating on the parameter server from the command line. It allows listing all parameters, and deleting, setting and getting individual parameter values. It also allows dumping and loading data dictionary to and from yaml file

### roscd: Navigating ROS Filesystem

`roscd` allows switching to package directory by only using the package name
```
> roscd package_name
```
`roscd` is one of the commands in the rosbash suite package. roscd is implemented as a shell bash shell function rather than an executable program. 

### rosed: Editing files
`rosed` allows editing a file in a certain ros package, like so
```
> rosed package_name file.cpp
```
The file will be opened in the editor specified by value of environment variable EDITOR

### rosrun: Starting a node
ROS node executables are typically not in the OS's search path for executables (i.e. the environment variable PATH). You could use rosrun to provide the package name and the node name like so
```
> rosrun package_name node_name
```
rosrun does not return until the node exits. So you can kill it by simply Ctrl-C. 

### roslaunch
`roslaunch` is a tool that reads XML description of a set of nodes, then launches and monitors those nodes. By	convention,	 roslaunch 	XML	files have	the	extension .launch and are called “launch files.”
    
### rostest: Testing a Multi-Node System
The rostest tool is just an extension to roslaunch, adding the <test> tag to allow specification of a test program to run alongside other nodes. After launching, the rest of the nodes, `rostest` will launch the test-node, this node is expected to use one of the standard testing frameworks to verify that the rest of the nodes are working properly and to report its finding in an xUnit-format output file

### rosnode, rostopic, rosmsg, rosservice and rossrv
These are bunch of command line tools to directly interrogate a master or a node about their state. 

### Debugging: /rosout and rqt_console
These tools allow us to check for error messages. The basic premise here is that you may not have access to a console that spawned a particular process. Like, in linux, there is log file somewhere where error messages are logged, however, ROS is a distributed framework, so processes may be logging their error information on log files on separate machines.


In ROS, a topic by the name /rosout is available, this topic carries messages of type Log. Any node can publish its error Logs on this topic, while any other node can subscribe to it. There is also a logging API in ros client library that makes it easy for nodes to publish the error Logs onto /rosout


**Reading the Log messages**</br>
To read error log messages, one can write a node which subscribes to /rosout. ROS however provides a GUI tool rqt_console, this is a much more convenient way of keeping track of error messages. 

`rqt_console` actually subscribes to `/rosout_agg`, here is the reason: A large ROS system has 100's of node running on multiple machines. To receieve those messages, rqt_console will need to establish a connect to each of those nodes, the total time may run into 10's of seconds which is unacceptable. The solution is to spawn a node (called `rosout`) when roscore is invoked, this node subscribes to /rosout, and republishes messages on an aggregated topic called /rosout_agg. rosout keep establishing connections with nodes as they are spawned by the system. Later when rqt_console starts, it need only make a single connection to rosout node over the /rosout_agg topic. 

### Debugging: rosnode and rqt_graph
These tools address problems relating to a missing connection or incorrect connections. The first step in debugging connections is to run `rqt_graph`. `rqt_graph` is a GUI that queries and visualizes nodes and topics. 


### Sensor Fusion: rviz
What if all the nodes are connected properly anf they are not raising any errors, but the robot is not behaving properly. It is a tool to visualize relevant sensor data from the robot. The details of what to visualize will depend on your application. 

### Plotting: rqt_plot
If you want to visualize individual values from a sensor, use `rqt_plot`. This tools supports 1D and 2D plotting of any numeric data that is published on ROS system. 
```
> rqt_plot /sin/data
```
The above example opens rqt_plot and plots the data published on the topic, /sin 's data field. 

### Data Logging and Analysis: rqt_bag and rosbag
It is common in ROS system to log data to file for later analysis. It basically works by subscribing a topic you want to log, and then writing it to disk. ROS provides a logging tools called `rosbag`. By convention, the resulting log files have extension `.bag`. These files are referred to as *ROS bags* or *bags*. 
```
> rosbag record -O log_file.bag /topic_name     # file name is optional, rosbag will generate a filename based on the timestamp
> rostopic echo /topic_name
> rosbag play log_file.bag
```
rosbag can also read the .bag file and play it back as shown above. 


rqt_bag is a convenient graphical tools for introspecting a bag file. Using	 rqt_bag ,you can see how many topics were recorded, how frequently messages on each topic were received, introspect contents of the messages, play back and loop over the entire bag or a section of it. 


## ROS Patterns

A collections of best practices for ROS development [here](http://wiki.ros.org/ROS/Patterns)


# ROS Course



