1. dependencies
- nav_msgs::Odometry
- imu_driver.h
- roscpp

2. Helper methods
- generate odometry message from acceleration (m/sec<sup>2</sup>) and gyroscope (radians/sec) measurements
```
void generate_odometry(const double & accMeas, const double & gyrMeas, nav_msgs::Odometry & odom)
```

3. Globals: none
3a. Constants: 
- FS_ACC
- FS_GYR

4. Inputs:
- argv[1]: node name
- argv[2]: imu model type, expected format imu_modeNum
- argv[3]: publish frequency

4a. 

5. Flow
- read command line inputs into argv
- perform inputs type and number error check, if check fails, return 1 with ROS ERROR message, log ROS ERROR
- init Node, with node name - argv[1]
- instantiate node handle

- instantiate Imu object with model type, argv[2]
- advertise publisher of type `nav_msgs::Odometry` using node handle

- instantiate nav_msgs::Odometry object, acceleration and gyro measurement arrays

- while (no termination signal)
  - read acceleration in accelration array
  - read gyro measurements in gyro array
  - generate odometry message using helper method
  - publishe odometry message
  - sleep for loop period

- release imu resources

6. Error Handling
- all Imu APIs will be invoked inside a try catch block. They are expected to return error string on exception. 
- In the catch block, we catch the std::exception, extract the error message, and print to stdout using ROS logging facility at ERROR level. We also log the message using ROS logging facility at ERROR level


