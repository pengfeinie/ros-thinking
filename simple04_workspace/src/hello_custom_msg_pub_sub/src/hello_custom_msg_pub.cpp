#include "ros/ros.h"
#include "hello_custom_msg_pub_sub/Person.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"talker_person");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<hello_custom_msg_pub_sub::Person>("chatter_person",1000);
    hello_custom_msg_pub_sub::Person p;
    p.name = "zhangsan";
    p.age = 2000;
    p.height = 1.45;

    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(p);
        p.age += 1;
        ROS_INFO("我叫:%s,今年%d岁,高%.2f米", p.name.c_str(), p.age, p.height);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}