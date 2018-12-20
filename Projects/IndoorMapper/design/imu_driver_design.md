1. Design Pattern: We will use the [factory pattern](https://www.oodesign.com/factory-pattern.html). 
- ratiale: we need to generate different product (i.e.) objects for each specific type of imu. This is so because the way these objects talk to their specific hardware is dependent on the imu's register api. On the other hand, the way we intend to use these imus in our application will be the same. 

- there we intend to create an interface that the client (i.e. ROS node) will use. We then create a concrete class that implements the interface. There can be multiple such concrete classes, one for each type of imu hardware. 

- we implement a imu_factory. Given an imu model name, it will instantiate an imu object from a set of allowable imu models using its concrete class, and pass the newly created object (casted as the abstract class i.e. the interface) to the client.


2. File types and content:
- there will be a imu_driver.h header file which will contain the method declarations, register address aliases. The header file will have define guards as per [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Header_Files)

- imu_driver.h is included at the end as per [style guideles](https://google.github.io/styleguide/cppguide.html#Name_and_Order_of_Includes)

- we have imu_driver.h that will include the abstract class definitions
- we will have imu9250.h that will have concrete class defintion


3. Namespaces
- using [style guide](https://google.github.io/styleguide/cppguide.html#Namespaces)
- we will wrap our abstract class (interface) and concrete class definitions inside namespaces


