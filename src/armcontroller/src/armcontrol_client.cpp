#include<ros/ros.h>
#include<serial/serial.h>
#include<armcontroller/goto_pos.h>//自定义服务goto_pos头文件
#include<armcontroller/relative_pos.h>//自定义服务relative_pos头文件
#include<armcontroller/pump.h>//自定义服务pump.srv头文件
#include<armcontroller/armcontrolMsg.h>//自定义消息头文件
#include<iostream>
/*依次打开客户端,服务端rosrun ...node,然后调用服务rosservice call ...x: y: z: */
int main(int argc,char** argv)
{
  ros::init(argc,argv,"armcontrol_client");
  ros::NodeHandle n;
    ros::NodeHandle n2;
      ros::NodeHandle n3;
  ros::ServiceClient client=n.serviceClient<armcontroller::goto_pos>("goto_pos");//goto_pos服务端
    ros::ServiceClient client2=n2.serviceClient<armcontroller::relative_pos>("relative_pos");//relative_pos服务端
      ros::ServiceClient client3=n3.serviceClient<armcontroller::pump>("pump");
  ros::service::waitForService("Service");
/*
  armcontroller::goto_pos goto_client;
  goto_client.request.x=80;
  goto_client.request.y=50;
  goto_client.request.z=80;
  client.call(goto_client);
  usleep(500*1000);

  armcontroller::relative_pos relative_client;
  relative_client.request.x2=50;
  relative_client.request.y2=20;
  relative_client.request.z2=10;
  client2.call(relative_client);

  //armcontroller::pump pump_client;
  //pump_client.request.v=1;
  //client3.call(pump_client);
*/
  return 0;
}
