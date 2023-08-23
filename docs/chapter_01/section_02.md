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





