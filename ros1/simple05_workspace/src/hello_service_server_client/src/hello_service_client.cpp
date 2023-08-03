#include "ros/ros.h"
#include "hello_service_server_client/AddInts.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    if (argc != 3)
    {
        ROS_ERROR("请提交两个整数");
        return 1;
    }
    ros::init(argc,argv,"AddInts_Client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<hello_service_server_client::AddInts>("AddInts");
    //
    ros::service::waitForService("AddInts");
    //
    // client.waitForExistence();
    hello_service_server_client::AddInts ai;
    ai.request.a = atoi(argv[1]);
    ai.request.b = atoi(argv[2]);
    bool flag = client.call(ai);
    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果:%d",ai.response.sum);
    }
    else
    {
        ROS_ERROR("请求处理失败....");
        return 1;
    }

    return 0;
}
