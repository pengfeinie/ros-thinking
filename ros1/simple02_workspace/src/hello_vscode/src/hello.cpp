#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello");
    ROS_INFO("Hello VSCode!!! 世界");
    return 0;
}