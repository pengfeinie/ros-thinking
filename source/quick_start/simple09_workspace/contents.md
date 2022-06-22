# hello world for custom header file 

## 1. background

Design the header file, the executable itself as the source file.

## 2. Tasks

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

**Step8:  compile**

ctrl + shift + B

![](images/2022-06-22_141154.png)

**Step9:  start** 

```
cd simple09_workspace
source ./devel/setup.bash
rosrun hello_custom_header hello
```

![](images/2022-06-22_142004.png)

**Reference：**

1. https://sir.upc.edu/projects/rostutorials/7-actions_tutorial/index.html?highlight=action
1. http://wiki.ros.org/actionlib_tutorials/Tutorials
1. http://wiki.ros.org/actionlib
1. http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
1. http://www.autolabor.com.cn/book/ROSTutorials/
