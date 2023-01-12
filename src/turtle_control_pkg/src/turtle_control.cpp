#include<ros/ros.h>  
#include<geometry_msgs/Twist.h>  //包含geometry_msgs所包含的Twist消息类型
#include<turtlesim/Pose.h>  //包含小乌龟位置信息消息头文件
#include<iostream>  
using namespace std;  


void callback(const turtlesim::Pose::ConstPtr& pose)  //Teleop具体内容
{   
    ROS_INFO("x:%.2f\r\ny:%.2f\r\ntheta:%.2f\r\nlinear:%.2f\r\nangular:%.2f\r\n",pose->x,pose->y,pose->theta,pose->linear_velocity,pose->angular_velocity);
}  
int main(int argc,char** argv)  //主函数
{  
    ros::init(argc, argv, "turtle_control");  //初始化ros
    ros::NodeHandle n; //定义ros句柄
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);//声明一个发布器，用于发布小乌龟控制速度
    ros::Subscriber sub = n.subscribe<turtlesim::Pose>("turtle1/pose",10,callback); //声明一个订阅器，用于订阅小乌龟位置信息
    geometry_msgs::Twist vel;
    sleep(1);
    vel.linear.x = 0.5;
pub.publish(vel);
    sleep(1);//前进1s
ros::spinOnce(); 

    vel.linear.x = -0.5;
    pub.publish(vel);
    sleep(1);//后退1s
ros::spinOnce();

    vel.linear.x = 0;
    vel.angular.z = 0.5;
    pub.publish(vel);
    sleep(1);//左转1s
ros::spinOnce();

    vel.linear.x = 0;
    vel.angular.z = -0.5;
    pub.publish(vel);
    sleep(1);//右转1s
ros::spinOnce();

    vel.linear.x = 0;
    vel.angular.z = 0;
    pub.publish(vel);//停止
    ros::spin(); 
    return 0;  
}
