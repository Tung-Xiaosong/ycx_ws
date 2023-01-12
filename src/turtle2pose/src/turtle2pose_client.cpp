#include <ros/ros.h>
#include <turtle2pose/position.h>

int main(int argc,char** argv)
{
 ros::init(argc,argv,"turtle_pose_client");
 ros::NodeHandle n;//定义一个句柄
 ros::ServiceClient client = n.serviceClient<turtle2pose::position>("pose_srv");//客户端定义与初始化
 ros::service::waitForService("pose_srv");

 turtle2pose::position pos_client;
 pos_client.request.x = 8;
 pos_client.request.y = 5;
 client.call(pos_client);

 pos_client.request.x = 2;
 pos_client.request.y = 8;
 client.call(pos_client);

 return 0;
}

