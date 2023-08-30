---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.11.5
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Chapter 01: Introduction to Robot Operating System (ROS)

## Section 023: ROS Service, service Node and Client

In this section, we covers the other one of the ROS basic communication channels **Service**, and how to provide or call services.

### ROS Service

The type of ROS Service are described by a `.srv` file, which is a simple text file that contains the fields of a ROS Service.
It contains an input message type, "---" and a output message type.
They are stored in the `./srv/` **directory** of a package and used to generate source code for services in different languages. 

`.srv`s are just simple text files with a field type and field name per line. The field types you can use are:

*  `int8`, `int16`, `int32`, `int64` (plus `uint*`)
*  `float32`, `float64`
*  `string`
*  `time`, `duration`
*  other msg files
*  variable-length `array[]` and fixed-length `array[C]` 

### Creating a msg in a ROS package

This part will instruct you create some ROS Service in a package.

From your ROS workspace folder, `workshop_demo_ws` etc, redirect to correct path:
```bash
cd src/
```

#### 1.Create Package
You can create the package with `message_generation` and `message_runtime` as dependency:
```bash
catkin_create_pkg <your package name> message_generation message_runtime
```

At build time, we need "message_generation", while at runtime, we only need "message_runtime". 
Fortunately, `catkin_create_pkg` will tell the difference and automatically specify different requirement by default.
Your `package.xml` should contains:
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

#### 2. Create Service

```bash
cd <your package name>
mkdir srv
```
Then, create your own service file in folder `srv`, such as `srv/MarkstoGrade.srv`:

```cmake
float32 marks
---
string grade
```

#### 3. Update CMakeLists.txt

The `catkin_create_pkg` should update your `CMakeLists.txt` as well:
```cmake
# Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

<!-- Also make sure you export the message runtime dependency. 
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
``` -->

You need to add your service files by:
```cmake
add_service_files(
  FILES
  MarkstoGrade.srv
)

```

Then, last step, you need to add:
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

#### 4. Build your Package
Now, you can build your package:

```bash
catkin build <your package name>
```

#### 5. Exam your Service
Then, do not forget to source:
```bash
source devel/setp.bash
```

And check your service by either using `rossrv list` to list all existing ROS messages:
```bash
rossrv list | grep <your service name>
```
or using `rosmsg package` to list all ROS services from one ROS package:
```bash
rossrv package <your package name>
```
You can also display the details (`-r`) in your service by:
```bash
rossrv show <package name>/<service name>
```



### Using ROS Service: Server and Clent

Communication on topics happens by server advertising the service and client calling the service. 
For the server and client to communicate, they must advertise and call the same type of message.

This part will instruct you create a ROS package that contains, some srv, some server and client in both python and c++.
> In addition: In large ROS project, once message/service type might be used by more than one ROS package. In this section, we are going to refer to the service in the package that we built in previous section.

From your ROS workspace folder, `workshop_demo_ws` etc, redirect to correct path:
```bash
source devel/setup.bash
cd src/
```

#### 1.Create Package

You can create the package with `message_generation` and `message_runtime` as dependency:
```bash
catkin_create_pkg <your package name> roscpp rospy std_msgs message_generation message_runtime <your adding service package name>
```

At build time, we need "message_generation", while at runtime, we only need "message_runtime". 
Fortunately, `catkin_create_pkg` will tell the difference and automatically specify different requirement by default.
Your `package.xml` should contains:
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```



#### 2. Create a cpp server node:
```cpp
#include "ros/ros.h"
#include "ros_demo_03_adding_srv/MarkstoGrade.h"

bool service_runner(ros_demo_03_adding_srv::MarkstoGrade::Request &req,
                    ros_demo_03_adding_srv::MarkstoGrade::Response &res)
                    
{
    res.grade = "N/A";
    if (req.marks >=80)
    {
        res.grade = "H1";
    }
    else if (req.marks >=75)
    {
        res.grade = "H2A";
    }
    else if (req.marks >=70)
    {
        res.grade = "H2B";
    }
    else if (req.marks >=65)
    {
        res.grade = "H3";
    }
    else if (req.marks >=50)
    {
        res.grade = "P";
    }else
    {
        res.grade = "N";
    }
    ROS_INFO("request: [marks: %f] received",req.marks);
    ROS_INFO("response: [grade: %s] sent", res.grade.c_str());
    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv, "marks2grade_server_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("marks_to_grade",service_runner);
    ROS_INFO("Server is ready");
    ros::spin();
    return 0;
}
```

#### 3. Create a python client node:
```python
#!/usr/bin/env python
from ros_demo_03_adding_srv.srv import MarkstoGrade
import rospy
import sys

service_name = "marks_to_grade"


def service_caller(marks):
    
    rospy.loginfo("waiting for service %s",service_name)
    rospy.wait_for_service(service_name)
    try:
        rospy.loginfo("Getting service now")
        service_runner = rospy.ServiceProxy(service_name,MarkstoGrade)
        rospy.loginfo("Calling service now")
        res = service_runner(marks)
        rospy.loginfo("The marks is %f, the grade is %s"%(marks,res.grade))
    except rospy.ServiceException:
         rospy.logwarn("service calling failed")
    

if __name__ == "__main__":

    # without the init, no ros log will get printed
    rospy.init_node("marks2fgrade_client_node")
    if len(sys.argv) == 2:
        marks = float(sys.argv[1])
    else:
        rospy.logerr("need exactly one argument for marks")
        sys.exit(1)
    
    service_caller(marks)
```




#### 8. Build your Package
Update your `CMakeLists.txt` file for your cpp node and python node.

Now, you can build your package:

```bash
catkin build <your package name>
```

#### 9. Exam your server node and client node
Then, do not forget to source:
```bash
source devel/setp.bash
```

You can run your client node first, it will wait for the server node advertising service.
Once you run your server node, the client node should be able to call the service and show the results.
In addition, you can also use `rosservice` to test your server node.

And check your service is advertised by using `rosservice list` to list all existing ROS messages:
```bash
rosservice list | grep <your service name>
```

You can also display the details about this topic by:
```bash
rosservice info <your service name>
rosservice args <your service name>
rosservice type <your service name>
```

You can also call the service with correct arguments:
```bash
rosservice call <your service name> <arguments>
```





