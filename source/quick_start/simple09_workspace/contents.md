# hello world for custom header file 

## 1. Hello world 1

### 1.1 Background

Design the header file, the executable itself as the source file.

### 1.2 Tasks

**Step1: create workspace and initialization**

```
mkdir -p simple09_workspace/src
cd simple09_workspace
catkin_make
```

![](images/2022-06-21_094305.png)

**Step2: start vscode**

```
cd simple09_workspace
code .
```

**Step3: compile ros in vscode**

using ***ctrl + shift + B*** to select ***catkin_make:build***

![](images/2022-06-21_133717.png)

**Step4: config tasks.json**

select ***Configure Default Build Task...*** , then please hit ***catkin_make:build***

![](images/2022-06-21_134117.png)

the task.json as below:

![](images/2022-06-22_135741.png)

**Step5: edit c_cpp_properties.json**

![](images/2022-06-22_140829.png)

**Step6: create ros package**

Selected src right click ---> create catkin package

![](images/2022-06-21_134154.png)

please type your package and dependencies.

```
hello_custom_header
roscpp rospy std_msgs
```

![](images/2022-06-21_134455.png)

**Step7: add hello.h in include/hello_custom_header folder**

```

namespace hello_ns{

class HelloPub {

public:
    void run();
};

}
```

![](images/2022-06-22_140506.png)

**Step8: add hello.cpp in src folder**

```
#include "ros/ros.h"
#include "hello_custom_header/hello.h"

namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    hello_ns::HelloPub helloPub;
    helloPub.run();
    return 0;
}
```

![](images/2022-06-22_140652.png)

**Step9: config CMakelists.txt**

```
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hello src/hello.cpp)

add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello
  ${catkin_LIBRARIES}
)
```

**Step10:  compile**

ctrl + shift + B

![](images/2022-06-22_141154.png)

**Step11:  start** 

```
cd simple09_workspace
source ./devel/setup.bash
rosrun hello_custom_header hello
```

![](images/2022-06-22_142004.png)

## 2. Hello world 2

### 2.1 Background

Design the header file with the source file and include the header file in the executable file.

### 2.2 Tasks

**Step1: create ros package**

Selected src right click ---> create catkin package

![](images/2022-06-21_134154.png)

please type your package and dependencies.

```
hello_custom_header_2
roscpp rospy std_msgs
```

![](images/2022-06-24_094634.png)

**Step2: add hello2.h in include/hello_custom_header_2 folder**

```
namespace hello_ns{

class HelloPub {

public:
    void run();
};

}
```

![](images/2022-06-24_094919.png)

**Step3: add hello2.cpp in src folder**

```
#include "hello_custom_header_2/hello2.h"
#include "ros/ros.h"

namespace hello_ns{

void HelloPub::run(){
    ROS_INFO("hello,head and src ...");
}

}
```

![](images/2022-06-24_095355.png)

**Step4: add use_header_main.cpp in src folder**

```
#include "ros/ros.h"
#include "hello_custom_header_2/hello2.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hahah");
    hello_ns::HelloPub my;
    my.run();
    return 0;
}
```

![](images/2022-06-24_095919.png)

**Step5: config CMakelists.txt**

```
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(head
  include/hello_custom_header_2/hello2.h
  src/hello2.cpp
)

add_dependencies(head ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head
  ${catkin_LIBRARIES}
)

add_executable(use_head src/use_head.cpp)

add_dependencies(use_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(use_head
  ${catkin_LIBRARIES}
)
```



**Reference：**

1. [https://sir.upc.edu/projects/rostutorials/7-actions_tutorial/index.html?highlight=action](https://sir.upc.edu/projects/rostutorials/7-actions_tutorial/index.html?highlight=action)
1. [http://wiki.ros.org/actionlib_tutorials/Tutorials](http://wiki.ros.org/actionlib_tutorials/Tutorials)
1. [http://wiki.ros.org/actionlib](http://wiki.ros.org/actionlib)
1. [http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
1. [http://www.autolabor.com.cn/book/ROSTutorials/](http://www.autolabor.com.cn/book/ROSTutorials/)
