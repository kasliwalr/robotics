
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
