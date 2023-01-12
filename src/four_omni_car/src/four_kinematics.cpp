//麦克纳姆四轮全向小车运动学
#include "ros/ros.h"
#include "fox_base/car_dataMsg.h"
#include "geometry_msgs/Twist.h"

float a=0.18;//底盘中心到车前的距离m0.1943  0.142
float b=0.108;//底盘中心到车边的距离0.1465  0.1775
float vw1,/*轮子1的线速度*/vw2,vw3,vw4;
float vtx/*X轴运动的速度*/,vty/*y方向移动速度*/,w/*自转的角速度*/;
float R=0.036;//车轮半径m0.04

ros::Subscriber m_robot_vel_sub;//订阅小车速度linear.x linear.y angular.z
ros::Publisher  m_robot_vel_pub;//发布小车速度
ros::Subscriber m_motor_speed_sub;//订阅驱动轮电机速度float64 speed[4]
ros::Publisher  m_motor_speed_pub;//发布电机速度
//PS:小车能自测得的是车轮电机转速,车轮速度需要转换
void four_vel_cb(const geometry_msgs::Twist::ConstPtr& four_vel)//订阅小车速度回调函数linear.x linear.y angular.z
{
  vtx=four_vel->linear.x;
  vty=four_vel->linear.y;
  w=four_vel->angular.z;
  //运动学反解,小车速度转换成电机转速
  vw1=vty-vtx+w*(a+b);//轮子1的线速度m/s
  vw2=vty+vtx-w*(a+b);
  vw3=vty-vtx-w*(a+b);
  vw4=vty+vtx+w*(a+b);

  vw1=60.0*vw1/2.0/3.14159/R;//r/min
  vw2=60.0*vw2/2.0/3.14159/R;
  vw3=60.0*vw3/2.0/3.14159/R;
  vw4=60.0*vw4/2.0/3.14159/R;

  fox_base::car_dataMsg four_m_cmd;
  four_m_cmd.speed[0]=-vw1;
  four_m_cmd.speed[1]=vw2;
  four_m_cmd.speed[2]=vw3;
  four_m_cmd.speed[3]=-vw4;
  m_motor_speed_pub.publish(four_m_cmd);
}

void four_motor_cb(const fox_base::car_dataMsg::ConstPtr& four_motor_vel)
{
  vw1=four_motor_vel->speed[0];
  vw2=four_motor_vel->speed[1];
  vw3=four_motor_vel->speed[2];
  vw4=four_motor_vel->speed[3];

  vw1=-vw1*2*3.14159*R/60.0;	//V=2PaiRN
  vw2=vw2*2*3.14159*R/60.0;
  vw3=vw3*2*3.14159*R/60.0;
  vw4=-vw4*2*3.14159*R/60.0;

  vtx=(vw4-vw1)/2;
  vty=(vw2+vw1)/2;
  w=(vw4-vw2)/2/(a+b);

  geometry_msgs::Twist four_m_real_vel;
  four_m_real_vel.linear.x=vtx;
  four_m_real_vel.linear.y=vty;
  four_m_real_vel.angular.z=w;
  m_robot_vel_pub.publish(four_m_real_vel);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"four_omni_car");
  ros::NodeHandle node;

  m_robot_vel_sub=node.subscribe<geometry_msgs::Twist>("four_cmd_vel",10,four_vel_cb);
  //rqt发布小车速度,这里订阅运动学逆解转换成电机转速,发布话题传向fox_base
  m_robot_vel_pub=node.advertise<geometry_msgs::Twist>("four_real_vel",10);
  //发布小车速度 作用:1.打印到终端	2.里程计接收小车速度用于计算里程

  m_motor_speed_sub=node.subscribe<fox_base::car_dataMsg>("car_data",10,four_motor_cb);
  //订阅fox_base发布的电机速度,通过正解发布小车速度
  m_motor_speed_pub=node.advertise<fox_base::car_dataMsg>("car_cmd",10);

  ros::spin();
  return 0;
}
