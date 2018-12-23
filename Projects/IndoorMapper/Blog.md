
## Dec 12th, 2018
My goal for the next few days is design, implement and test the IMU module. The ROS navigation stack as depicted here, requires information to estimate the pose of the robot. This is accomplishes using odometry from IMU sensor and wheel encoders. Using the velocity/acceleration information from these sensors, the robot will be able to estimate its position (base_link) with respect to the odom frame which is the indoor room reference. 

The navigation stack requires that `nav_msgs/Odometry` messages by published over ROS. Below is the message structure
```
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```
The pose corresponds to the robot's estimated pose in `odom` frame with optional covariance information. The twist corresponds to robot's velocity in child frame (normally framed associated with mobile base), this too with optional covariance informaiton. 
The PoseWithCovariance type contains 6 elements for the 6 DOFs and 36 elements array for storing the covariance matrix. Same goes for TwistWithCovariance.  The calculation of pose's six DOFs is simple enough if you know the velocity along the different DOFs. Just note that apporpriate transformations are in order to be able to obtain pose in `odom` frame based on velocities in `mobile base` frame. 


Ok, but we need to dig a little deeper. How does navigation stack use this information? Are there any requirements on the quality of information? 
For now we'll assume that all sensors are working properly (we'll get to verifying the assumption later). How does the navigation stack use this information. It uses it two ways - for map building and for navigation. Map building precedes navigation in our case and both requires the use of the sensor array. 

Two packages are often employed for this purpose - gmapping for map building and AMCL for navigation. 
Lets also explore other players. 
- costmap_2d package: provides costmaps for use by navigation algorithms. It publishes two topics of message types 
  - `nav_msgs/OccupancyGrid`: This represents a 2-D grid map, in which each cell represents the probability of occupancy. 
  - `map_msgs/OccupancyGridUpdate: 
The costmap_2d package receives sensor information from sensor streams, but needs tf transform information to make sense of it 
- global_planner: its API allows the caller to do 3 things
  - initialize the planner with costmap
  
  
  So basically a lot of information is being transferred from one place to the other. Lets tablulate who is talking to whom
  
  
  - makePath(start, goal, plan), plan is passed as reference. start, goal and plan are of type `geometry_msgs/PoseStamped`. 
  - one can in principle use any algorithm for planning. The package provides some algorithms which includes graph search algorithms such a Dijkstra and A*. [Here is more on A*'s use in robotics](https://www.coursera.org/lecture/robotics-motion-planning/1-4-a-algorithm-Vv9fL)
  - one can specify the algorithms by setting use_dijkstra parameter. 
  - global_planner publishes message of type `nav_msgs/Path` which is basically an array of poses. this can be used for display purposes. 
- base_local_planner
  - This package provides implementations of the [Trajectory Rollout](https://pdfs.semanticscholar.org/dabd/bb636f02d3cff3d546bd1bdae96a058ba4bc.pdf?_ga=2.75374935.412017123.1520536154-80785446.1520536154) and [Dynamic Window](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) approaches to local robot navigation on a plane. 
  - it publishes both global and local plan for visualization purposes. it subscribes to `nav_msgs/Odometry`
  

The need for local planner is to avoid obstacles that often come up but are not on the map. This is a realistic situation. For example an robot nvigating indoors will encounter a walking person, a child, some toys lying on the ground and such. 
  
Now two more packages need to be understood, before we move on
- amcl: to navigate a robot needs to localize itself. the global planner is give a map and a goal, and it generates an optimal path. The local planner somehow manages to map a portion of a global map onto its position and then generates optimal trajectories in that local map. But how does it know which portion of the global map is relevant to it, in other words it has to know where is it on the global map - it has to localize itself. This is done for it by amcl. 
  - amcl implements [Monte Carlo Localization](http://robots.stanford.edu/papers/fox.aaai99.pdf) approach
  - several other modesl are employed from Probabilistic Robotics book - sample_motion_model_odometry, beam_range_finder_model, likelihood_field_range_finder_model, Augmented_MCL, and KLD_Sampling_MCL.
  - amcl takes in a laser-based map, laser scans, and transform messages, and outputs pose estimates. 
- mapping: This package contains a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping
  - it generates an occupancy grid based map using laser-scan data that it subscribes to. 
  - [main ref paper](http://ais.informatik.uni-freiburg.de/publications/papers/grisetti07tro.pdf)
  

Secondly, we also need to figure out what information is available to use for velocity and position from the IMU and encoders and in what frame?


The [navigation tuning guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide) might give us some clues. 


## Dec 14th, 2018
### TF2
At the minimum we have the following frames associated with our robot
- map_frame
- odom_frame
- base_link_frame
- lidar_frame
- imu_frame

Some node/nodes have to publish information so that the TF tree can be build. On the listener side, one excepts that a TF tree be in place and frame of certain type and names be present, so that their relative transform information can be extracted. 

- imu_frame: the acceleration and rotational velocity information that is generated by the IMU is wrt the IMU axes. What we are interested in is the acceleration and rotational velocity wrt the base_link_frame. This relationship between the IMU_frame and base_link frame is static. Therefore we need to broadcast the static transform base_link_frame ---> imu_frame
- lidar_frame: the distance and angle information generated by the lidar is wrt the lidar_frame. What we are interested in the the distance and angular information in the base_link_frame. Therefore we need to broadcast the static transform base_link_frame ---> lidar frame. 
- odom_frame: it is a world_fixed_frame. The robot's position in this frame is infrerred from the acceleration and velocity of the base_link_frame. Given the initial relationship between odom_frame and base_link_frame, one can estimate the where the base_link is wrt odom origin. Therefore we need to broadcast this dynamic transform, odom_frame ---> base_link_frame. 
- map_frame: the map frame and odom frame overlap each other. It should be noted however that robot's position in map_frame is determined using the lidar. amcl publishes the transofrm odom_frame ---> map_frame. 

### How to publish the nav_msgs/Odometry? 
Here is how to populate the Odometry msg
```
std_msgs/Header header
  uint32 seq
  time stamp: now()
  string frame_id: coordinate frame used for pose
string child_frame_id: coordinate frame used for velocity
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```


## Dec 18th, 2018

I was busy doing some other stuff. So carrying on from before, here is a list of things that need to be done to finish the IMU module. But first let me remind myself, what the structure of IMU module is going to be. 

IMU module will consists of a ROS node. We could provide certain parameters to it such as which module of IMU to run. The ROS node will then start transmitting messages onto the Odometry frame. The IMU node will call on the C++ IMU library that we'll implement and test. 

At the C++ IMU library level:

1. I tested that self test is working when running IMU from Arduino. But I faulted in my own calculations in following the designated protocol. So I'll try and replicate the code on raspberry Pi and rerun the test.

2. Write C++ IMU API. The user for this API will be the ROS node. We would need to finalize the ROS API first then, so that we are clear about the IMU API. 

3. The C++ IMU API should be an class, with the public methods as the API. The private methods will call on Pigpio lib to do the dirty work i.e. discovering and communicating with the IMU

4. Since we want to run multiple IMUs, we expect ROS API to receive IMU model from the command line/launch file as constructor argument. The class implementation will be such that based on the model, it will instantiate appropriate object type. We will implement IMU classes that implement an C++ IMU interface. However each individual class will talk to a specific IMU module type in a custom manner. 

5. Don't expect multiple IMUs running on the same platform. 


Here is a bucket list of items

- port self test to RPi
- finalize ROS node API
- finalize IMU C++ API
- write IMU C++ API unit tests & mocks
- implement API. test and repeat
- write IMU ROS unit tests 
- perform ROS node integration testing
- documentation


## Dec 19th, 2018

Ok, so I finished the first part of self test. Here is what I did

1. I ran the self test on Arduino setup - Arduino Uno connected to PC via virtual serial port, IMU9250 connected to arduino. Then I ran the sparkfun sketch_dec05a. This resulted in initiation of reading from the sensor. The code first performs a self test, the self test generates %age deviation from factory setting. The settings reported by this system is < +/-5% from the factory setting. 
```
x-axis self test: acceleration trim within : -0.0% of factory value
y-axis self test: acceleration trim within : -2.9% of factory value
z-axis self test: acceleration trim within : -1.6% of factory value
x-axis self test: gyration trim within : -0.9% of factory value
y-axis self test: gyration trim within : -0.3% of factory value
z-axis self test: gyration trim within : -0.5% of factory value
```


2. On inspection of the IMU9250 SelfTest class method, I found a few changes worth making. I made the change to my test_imu c++ code. And reran the test. 
```
gpio initialized
I2C status 0
224
gyrx: -0.181269%
gyry: -0.991401%
gyrz: -0.318571%
accx: -0.152679%
accy: 2.34237%
accz: -1.67486%
```

These both look comparable. 

## Dec 23th, 2018
After some delay I come back here. I had been reorganizing my repositories and doing some planning on subject areas that I need to work on. I did get some work done on the IMU node design. I specified the IMU ROS node's API and the IMU driver library API. I then worked on the design of the IMU driver and IMU ROS node. Both of these are specified in the design document [here](design/Design.md/#imu-module)

The IMU ROS node will call the IMU driver API to talk to the IMU. The ROS node's job is to properly format the data from the IMU driver node and publish it to the *nav_msgs/Odometry* topic. 

The IMU ROS node is simple. There is a main() inside which we run an infinite loop which publishes odometry data. There may be some helper functions to perform IMU driver output translation to *Odometry*. 

The IMU driver design is little more involved. The requirement from the design is as follows
- driver should allow transparent and consistent interface to read and control the IMU
- driver should be able to handle different types of IMUs

To accomplish this task we choose the following design
- an abstract base class that defines the IMU interface
- each imu class implements the abstract class interface by inherting from the base class. 
- there is a imu factory that that return a specific imu class based on the *model_type* string passed to its *createIMU()* method






