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

## Section 02: ROS Message, topic, publisher and subscriber

In this section, we covers one of the ROS basic communication channel **Message**, and how to "send" or "receive" messages.

### ROS Message

The type of ROS Message are described by a `msg` file, which is a simple text file that contains the fields of a ROS message. They are stored in the `./msg/` **directory** of a package and used to generate source code for messages in different languages. 

msgs are just simple text files with a field type and field name per line. The field types you can use are:

*  `int8`, `int16`, `int32`, `int64` (plus `uint*`)
*  `float32`, `float64`
*  `string`
*  `time`, `duration`
*  other msg files
*  variable-length `array[]` and fixed-length `array[C]` 

There is also a special type in ROS: `Header`, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have `Header header`. 
Here is an example:

```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```


### Creating a msg in a ROS package

This part will instruct you create some ROS Message in a package.

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

#### 2. Create Message

```bash
cd <your package name>
mkdir msg
```
Then, create your own message file in folder `msg`, such as `msg/StudentInfo.msg`:

```cmake
string first_name
string last_name
uint8 age
# between 0-100
uint32 score
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

You need to add your message files by:
```cmake
add_message_files(
  FILES
  <your_message_name>.msg
  # Do not include path
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

#### 5. Exam your Message
Then, do not forget to source:
```bash
source devel/setp.bash
```

And check your message by either using `rosmsg list` to list all existing ROS messages:
```bash
rosmsg list | grep <your message name>
```
or using `rosmsg package` to list all ROS messages from one ROS package:
```bash
rosmsg package <your package name>
```
You can also display the details (`-r`) in your message by:
```bash
rosmsg show <package name>/<message name>
```



### Using ROS Message: Topic, publisher and subscriber

Communication on topics happens by sending ROS messages between nodes.
For the publisher and subscriber to communicate, the publisher and subscriber must send and receive the same type of message.

This part will instruct you create a ROS package that contains, some msg, some publisher and subscriber in both python and c++.

From your ROS workspace folder, `workshop_demo_ws` etc, redirect to correct path:
```bash
cd src/
```

#### 1.Create Package
You can create the package with `message_generation` and `message_runtime` as dependency:
```bash
catkin_create_pkg <your package name> roscpp rospy std_msgs message_generation message_runtime
```

At build time, we need "message_generation", while at runtime, we only need "message_runtime". 
Fortunately, `catkin_create_pkg` will tell the difference and automatically specify different requirement by default.
Your `package.xml` should contains:
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

#### 2. Create Message

```bash
cd <your package name>
mkdir msg
```
Then, create your own message file in folder `msg`, such as `msg/StudentInfo.msg`:

```cmake
string first_name
string last_name
uint8 age
# between 0-100
uint32 score
```

#### 3. Create a python publisher with ROS builtIn message type:
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # the topic name is chatting
    # the Message type is builtIn String
    pub = rospy.Publisher('chatting', String, queue_size=10)
    rospy.init_node('rospy_string_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### 4. Create a python subscriber with ROS builtIn message type:
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rospy_string_subscriber', anonymous=True)

    rospy.Subscriber("chatting", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```


#### 5. Create a python publisher with customised message type:
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from ros_demo_02_using_msg.msg import StudentInfo



def talker():
    pub = rospy.Publisher('student_details', StudentInfo, queue_size=10)
    rospy.init_node('rospy_StudentInfo_publisher', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        student_info = StudentInfo()
        student_info.first_name = "Guang"
        student_info.last_name = hello_str
        student_info.age = 18
        # student_info
        rospy.loginfo(student_info)
        pub.publish(student_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

```



#### 6. Create a C++ subscriber with customised message type:
```cpp
#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "ros_demo_02_using_msg/StudentInfo.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const ros_demo_02_using_msg::StudentInfo::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->last_name.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "roscpp_StudentInfo_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("student_details", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

```

#### 7. Update CMakeLists.txt

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

You need to add your message files by:
```cmake
add_message_files(
  FILES
  <your_message_name>.msg
  # Do not include path
)

```

Then, last step, you need to add:
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

#### 8. Build your Package
Now, you can build your package:

```bash
catkin build <your package name>
```

#### 9. Exam your Message
Then, do not forget to source:
```bash
source devel/setp.bash
```

And check your message by either using `rosmsg list` to list all existing ROS messages:
```bash
rosmsg list | grep <your message name>
```
or using `rosmsg package` to list all ROS messages from one ROS package:
```bash
rosmsg package <your package name>
```
You can also display the details (`-r`) in your message by:
```bash
rosmsg show <package name>/<message name>
```




