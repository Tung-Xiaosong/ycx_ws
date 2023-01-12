//实验9:三轮全向小车
//运动学正逆解,使小车运动
#include "ros/ros.h"
#include "fox_base/car_dataMsg.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
using namespace std;

float L = 0.26; //轮子到底盘中心之间的距离
float R = 0.04;//轮子半径
float delta=60;//轮子3与Y轴正方向夹角60度

//定义全局发布,订阅器
ros::Subscriber m_robot_vel_sub;//订阅小车速度linear.x angular.z
ros::Publisher  m_robot_vel_pub;//发布小车速度
ros::Subscriber m_motor_speed_sub;//订阅驱动轮电机速度float64 speed[4]
ros::Publisher  m_motor_speed_pub;//发布电机速度
//PS:小车能自测得的是车轮电机转速,车轮速度需要转换
void three_vel_cb(const geometry_msgs::Twist::ConstPtr& three_vel)//订阅小车速度回调函数linear.x angular.z
{
  float v1/*一轮线速度*/,v2/*二轮线速度*/,v3/*三轮线速度*/;
  float vx/*沿坐标系x方向移动速度*/,vy/*y方向移动速度*/,vth/*机器人绕自身中心旋转速度*/;

  vx=three_vel->linear.x;
  vy=three_vel->linear.y;
  vth=three_vel->angular.z;
  //运动学逆解,小车速度转换成电机转速
  v1=-vx*sqrt(3)/2-vy/2-L*vth;//轮1的线速度m/s
  v2=vx*sqrt(3)/2-vy/2-L*vth;//轮2的线速度
  v3=vy-L*vth;//轮3的线速度

  v1=60.0*v1/2.0/3.14159/R;//电机1转速rad/min
  v2=60.0*v2/2.0/3.14159/R;//符号!!!!!!!!!!
  v3=60.0*v3/2.0/3.14159/R;

  fox_base::car_dataMsg three_m_cmd;//自定义消息car_dataMsg变量
  three_m_cmd.speed[0] = v1;//三个轮分别接1,2,3电机
  three_m_cmd.speed[1] = v2;
  three_m_cmd.speed[2] = v3;
  m_motor_speed_pub.publish(three_m_cmd);//通过话题car_cmd发布电机速度给fox_base使电机转动
}

void three_motor_cb(const fox_base::car_dataMsg::ConstPtr& three_motor_vel)//订阅驱动轮电机速度回调函数float64 speed[4]
{
  float vx/*小车x轴线速度*/,vy/*小车y轴线速度*/,vth/*机器人绕自身中心旋转速度*/;
  float v1/*一轮线速度*/,v2/*二轮线速度*/,v3/*三轮线速度*/;

  v1 = three_motor_vel->speed[0];//1轮电机转速rad/min
  v2 = three_motor_vel->speed[1];//2轮电机转速rad/min
  v3 = three_motor_vel->speed[2];//3轮电机转速rad/min

  v1 = v1*2*3.14159*R/60.0;//1轮线速度m/s,V=2paiRn
  v2 = v2*2*3.14159*R/60.0;//2轮线速度m/s
  v3 = v3*2*3.14159*R/60.0;//3轮线速度m/s
//运动学正解,将电机速度转换成小车速度
  vx=-v1/sqrt(3)+v2/sqrt(3);
  vy=-v1/3-v2/3+2*v3/3;
  vth=-v3/3/L-v2/3/L-v3/3/L;

  geometry_msgs::Twist three_m_real_vel;//Twist消息变量m_real_el
  three_m_real_vel.linear.x = vx;
  three_m_real_vel.linear.y=vy;
  three_m_real_vel.angular.z = vth;
  m_robot_vel_pub.publish(three_m_real_vel);//通过订阅话题car_data发布小车速度
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "three_fox_kinematics");
  ros::NodeHandle m_h;
/***********************************************************/
  //cmd的都是给速度的命令
  //小车速度
  m_robot_vel_sub = m_h.subscribe<geometry_msgs::Twist>("three_cmd_vel",10,three_vel_cb);//订阅小车速度linear.x angular.z
  m_robot_vel_pub = m_h.advertise<geometry_msgs::Twist>("three_real_vel",10);//发布小车速度linear.x angular.z
  //电机速度
  /*
    三轮小车和fox_base节点(驱动轮子电机)之间通过car_data和car_cmd话题建立联系,
    三轮小车发布car_cmd话题给电机发布速度float64[4] speed
    三轮小车订阅car_data话题接收电机传来的电机速度float64[4] speed
  */
  m_motor_speed_sub = m_h.subscribe<fox_base::car_dataMsg>("car_data",10,three_motor_cb);//订阅驱动轮电机速度float64 speed[4]
  m_motor_speed_pub = m_h.advertise<fox_base::car_dataMsg>("car_cmd",10);//发布电机速度float64 speed[4]
/***********************************************************/
  ros::spin();
  return 0;
}

