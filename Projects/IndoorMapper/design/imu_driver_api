### IMU DRIVER API

1. namespace
- all imu classes are part of imu_driver namespace


2. instantiation
- instantiates an imu object of specific model type
```
try{
    imu_driver::Imu(string & imu_modelnum);
}
catch(){
 //print and log error information using ROS logging
}
```
- parameter values will be optional, default value is imu_9250
- all public member methods of class Imu return void. They manage error information using exceptions

3. initialize imu
- initializes the imu - selftest followed by calibration 
- optional parameters can be passed. will be decided at a later date
```
Imu.init(param1, param2.....);
```

4. read accelerometer
- read x, y, z acceleration. units m/sec<sup>2</sup>
```
//imu is object of type Imu
imu.readAcc(float *Acc);     //Acc is pointer to first element of a 3 element double array. Acc[0]: x-dir, Acc[1]: y-dir, Acc[2]: z-dir
```

5. read gyroscope
- read x,y,z axis rotational velocity. units radians/sec
```
//imu is object of type Imu
imu.readGyr(float *Gyr);    //Gyr is pointer to first element of a 3 element double array. Gyr[0]: x-rotation, Gyr[1]: y-rotation, Gyr[2]: z-rotation
```

6. shutdown
- aborts connection to imu over i2c. imu is released for use by another process
```
imu.shutdown()
```

