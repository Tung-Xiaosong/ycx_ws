#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>

void callback(const geometry_msgs::Twist::ConstPtr&msg)
{
	ROS_INFO("x=%f,y=%f,z=%f,angular z=%f",msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.z);
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"turtle_pose");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe<geometry_msgs::Twist/*turtlesim::Pose*/>("/turtle1/cmd_vel"/*/turtle1/pose*/,10,&callback);
	ros::spin();
	return 0;
}
