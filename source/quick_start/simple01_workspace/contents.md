# hello world

**Step1: create workspace and initialization**

```
mkdir -p simple01_workspace/src
cd simple01_workspace
catkin_make
```

![](images/2022-06-09_125711.png)

**Step2: create dependency package**

```
cd src
catkin_create_pkg roscpp rospy std_msgs
```

![](images/2022-06-09_144327.png)

The above command will generate a function package in the workspace, which depends on roscpp, rospy and std_msgs, where roscpp is a library implemented in C++, rospy is a library implemented in python, and std_msgs is a standard message library. Create When ROS function packages are used, they generally rely on these three libraries for implementation.

**Step3: add hello.cp in src/hello folder**

```
mkdir hello
cd hello
```

```c++
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hello");
    ros::NodeHandle n;
    ROS_INFO("hello world!");
    return 0;
}
```

![](images/2022-06-09_154538.png)

**Step4: update CMakelist.txt**

Before we build the package, you can add the following few lines to the bottom of your CMakeLists.txt. Or you can search the cmake functions to make changes. The second ways is preferred as you gain more experience with ROS. But the first method is fine.

```
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(hello hello/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})
```

![](images/2022-06-09_161513.png)

**Step5: make**

```
cd simple01_workspace
catkin_make
```

**Step6:  start roscore**

```
roscore
```

**Step7:  start hello**

```
cd simple01_workspace
source ./devel/setup.bash
rosrun hello hello
```



**Reference：**

1. http://wiki.ros.org/action/fullsearch/catkin/commands/catkin_make
