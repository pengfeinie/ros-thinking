#include "ros/ros.h"
#include "hello_custom_header_2/hello2.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hahah");
    hello_ns::HelloPub my;
    my.run();
    return 0;
}
