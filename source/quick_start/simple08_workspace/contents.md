# hello world for turtlesim 

## 1. background

This tutorial introduces ROS graph concepts and discusses the use of [roscore](http://wiki.ros.org/roscore), [rosnode](http://wiki.ros.org/rosnode), and [rosrun](http://wiki.ros.org/rosrun) commandline tools.

## 2. Control turtlesim using publisher

### 2.1 Topic and Message 

So now we can run the turtlesim_node in the turtlesim package.

**Step1: Start rosrun :**

```
roscore
```

**Step2: Then, in a new terminal:**

```
rosrun turtlesim turtlesim_node
```

![](images/2022-06-19_135532.png)

**Step3: list topic**

```
rostopic list
```

![](images/2022-06-19_135940.png)

**Step4: message**

```
rostopic type /turtle1/cmd_vel
rosmsg info geometry_msgs/Twist
```

![](images/2022-06-19_140250.png)

### 2.2. Tasks

**Step1: create workspace and initialization**

```
mkdir -p simple08_workspace/src
cd simple08_workspace
catkin_make
```

![](images/2022-06-19_140431.png)

**Step2: start vscode**

```
cd simple08_workspace
code .
```

**Step3: compile ros in vscode**

using ***ctrl + shift + B*** to select ***catkin_make:build***

![](images/2022-06-19_140556.png)

**Step4: config tasks.json**

select ***Configure Default Build Task...*** , then please hit ***catkin_make:build***

![](images/2022-06-19_140917.png)

the task.json as below:

![](images/2022-06-19_140734.png)

**Step5: create ros package**

Selected src right click ---> create catkin package

![](images/2022-06-19_141022.png)

please type your package and dependencies.

```
hello_turtlesim
roscpp rospy std_msgs geometry_msgs
```

![](images/2022-06-19_141157.png)

**Step6: add hello_turtlesim_controller.cpp in src folder**

```
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello_turtlesim_controller");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 2.0;

    ros::Rate r(10);
    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}
```

![](images/2022-06-19_142842.png)

**Step7: config CMakelists.txt**

```
add_executable(hello_turtlesim_controller src/hello_turtlesim_controller.cpp)

target_link_libraries(hello_turtlesim_controller
  ${catkin_LIBRARIES}
)
```

**Step8:  compile**

ctrl + shift + B

![](images/2022-06-19_142940.png)

**Step9:  start** 

```
roscore

rosrun turtlesim turtlesim_node

cd simple08_workspace
source ./devel/setup.bash
rosrun hello_turtlesim hello_turtlesim_controller
```

![](images/2022-06-19_142654.png)

<video width="700" controls>
	<source src="/en/latest/_static/hello_turtlesim_controller.mp4" />
</video>


## 3. Display turtlesim track data using subscriber

### 3.1 Topic and Message 

So now we can run the turtlesim_node in the turtlesim package.

**Step1: Start rosrun :**

```
roscore
```

**Step2: Then, in a new terminal:**

```
rosrun turtlesim turtlesim_node
```

![](images/2022-06-19_135532.png)

**Step3: list topic**

```
rostopic list
```

![](images/2022-06-19_135940.png)

**Step4: message**

```
rostopic type /turtle1/pose
rosmsg info turtlesim/Pose
```

![](images/2022-06-19_150556.png)

### 3.2 Tasks

**Step1: add hello_turtlesim_display_track.cpp in src folder**

```
#include "ros/ros.h"
#include "turtlesim/Pose.h"

void doPose(const turtlesim::Pose::ConstPtr& p){
    ROS_INFO("乌龟位姿信息:x=%.2f,y=%.2f,theta=%.2f,lv=%.2f,av=%.2f",
        p->x,p->y,p->theta,p->linear_velocity,p->angular_velocity
    );
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"sub_pose");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅者对象
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,doPose);
    // 5.回调函数处理订阅的数据
    // 6.spin
    ros::spin();
    return 0;
}
```

![](images/2022-06-19_151120.png)

**Step2: config CMakelists.txt**

```
add_executable(hello_turtlesim_display_track src/hello_turtlesim_display_track.cpp)

target_link_libraries(hello_turtlesim_display_track
  ${catkin_LIBRARIES}
)
```

**Step3:  compile**

ctrl + shift + B

![](images/2022-06-19_151243.png)

**Step4:  compile**

```
roscore

rosrun turtlesim turtlesim_node

cd simple08_workspace
source ./devel/setup.bash
rosrun hello_turtlesim hello_turtlesim_controller

cd simple08_workspace
source ./devel/setup.bash
rosrun hello_turtlesim hello_turtlesim_display_track
```

![](images/2022-06-20_095554.png)

**Reference：**

1. https://sir.upc.edu/projects/rostutorials/7-actions_tutorial/index.html?highlight=action
1. http://wiki.ros.org/actionlib_tutorials/Tutorials
1. http://wiki.ros.org/actionlib
1. http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
