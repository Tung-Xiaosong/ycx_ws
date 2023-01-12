//实验8:两轮差速小车
//运动学正逆解,使小车运动
#include "ros/ros.h"
#include "fox_base/car_dataMsg.h"
#include "geometry_msgs/Twist.h"
using namespace std;

float L = 0.35; //两驱动轮间的距离
float R = 0.03725;//轮子半径

//定义全局发布,订阅器
ros::Subscriber m_robot_vel_sub;//订阅小车速度linear.x angular.z
ros::Publisher  m_robot_vel_pub;//发布小车速度
ros::Subscriber m_motor_speed_sub;//订阅驱动轮电机速度float64 speed[4]
ros::Publisher  m_motor_speed_pub;//发布电机速度
//PS:小车能自测得的是车轮电机转速,车轮速度需要转换
void vel_cb(const geometry_msgs::Twist::ConstPtr& vel)//订阅小车速度回调函数linear.x angular.z
{
  float vx/*小车线速度*/,vth/*小车角速度*/,vl/*左轮电机转速*/,vr/*右轮电机转速*/;

  vx = vel->linear.x;//小车线速度
  vth = vel->angular.z;//小车角速度
//运动学逆解
  vl = vx - vth*L/2;//左轮速度m/s
  vr = vx + vth*L/2;//右轮速度m/s
  vl = 60.0*vl/2.0/3.14159/R;//左轮电机转速rad/min,线速度和转速关系:V=2paiRn--->n=V/2paiR
  vr = -(60.0*vr/2.0/3.1415926/R);//右轮电机转速rad/min,前面加上负号,才能使两个轮同一方向转动

  fox_base::car_dataMsg m_cmd;//自定义消息car_dataMsg变量
  m_cmd.speed[0] = vl;//左右轮分别接在1,4电机上
  m_cmd.speed[3] = vr;
  m_motor_speed_pub.publish(m_cmd);//发布电机速度
}

void motor_cb(const fox_base::car_dataMsg::ConstPtr& motor_vel)//订阅驱动轮电机速度回调函数float64 speed[4]
{
  float vx/*小车线速度*/,vth/*小车角速度*/,vl/*左轮电机转速*/,vr/*右轮电机转速*/;

  vl = motor_vel->speed[0];//左轮电机转速rad/min
  vr = motor_vel->speed[3];//右轮电机转速rad/min
  vr = -vr;//换向
  vl = vl*2*3.14159*R/60.0;//左轮速度m/s,V=2paiRn
  vr = vr*2*3.14159*R/60.0;//右轮速度m/s
//运动学正解
  vx = (vl+vr)/2.0;//小车线速度
  vth = (vr-vl)/L;//小车角速度

  geometry_msgs::Twist m_real_vel;//Twist消息变量m_real_el
  m_real_vel.linear.x = vx;
  m_real_vel.angular.z = vth;
  m_robot_vel_pub.publish(m_real_vel);//发布小车速度
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fox_kinematics");
  ros::NodeHandle m_h;
/***********************************************************/
  //cmd的都是给速度的命令
  //小车速度
  m_robot_vel_sub = m_h.subscribe<geometry_msgs::Twist>("cmd_vel",10,vel_cb);//订阅小车速度linear.x angular.z
  m_robot_vel_pub = m_h.advertise<geometry_msgs::Twist>("real_vel",10);//发布小车速度linear.x angular.z
  //电机速度
  m_motor_speed_sub = m_h.subscribe<fox_base::car_dataMsg>("car_data",10,motor_cb);//订阅驱动轮电机速度float64 speed[4]//e
  m_motor_speed_pub = m_h.advertise<fox_base::car_dataMsg>("car_cmd",10);//发布电机速度float64 speed[4]
/***********************************************************/
  ros::spin();
  return 0;
}
