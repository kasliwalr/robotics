
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
  - 

Secondly, we also need to figure out what information is available to use for velocity and position from the IMU and encoders and in what frame?


The [navigation tuning guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide) might give us some clues. 
