#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtle2pose/position.h>
#include <iostream>
#include <unistd.h>
class turtlepose
{
 public:
 	turtlepose();
 private:
 	ros::NodeHandle n;
 	ros::Subscriber sub;
 	ros::Publisher pub;
 	ros::ServiceServer service;
 void callback(const turtlesim::Pose::ConstPtr& pose);//订阅回调函数
 bool callbackSrv(turtle2pose::position::Request& req, turtle2pose::position::Response& res);//服务回调函数
 float pose_x,pose_y,pose_z;
 float error_x,error_y,error_z,error,error1;
 double vl,va;
};
turtlepose::turtlepose()
{
	sub=n.subscribe<turtlesim::Pose>("/turtle1/pose",10,&turtlepose::callback,this);
 	pub=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
 	service=n.advertiseService("pose_srv",&turtlepose::callbackSrv,this);
}
void turtlepose::callback(const turtlesim::Pose::ConstPtr& pose)//订阅回调具体内容
{
 	pose_x=pose->x;//将 pose 消息的 x 赋值给变量 pose_x
 	pose_y=pose->y;//将 pose 消息的 y 赋值给变量 pose_y
 	pose_z=pose->theta;//将 pose 消息的 theta 赋值给 pose_z
 }
bool turtlepose::callbackSrv(turtle2pose::position::Request& req,
 turtle2pose::position::Response& res)//服务回调
{
 	geometry_msgs::Twist v; //一个速度的消息格式,假定做匀速直线运动，设定运行时间为 3s
 	float goal = atan2((req.y-pose_y),(req.x-pose_x));
 	ROS_INFO("goal:%.2f",goal);
 while(1)
 {
 	 error_z = atan2((req.y-pose_y),(req.x-pose_x))-pose_z;
 	 if(error_z>3.14159)error_z-=6.28319;//角度做一个定义域处理
 	 if(error_z<-3.14159)error_z+=6.28319;
 	 error_x = (req.x-pose_x)*(req.x-pose_x)+(req.y-pose_y)*(req.y-pose_y);
 	 error_x = sqrt(error_x);
  //ROS_INFO("error_z:%.2f",error_z);
 	 if(fabs(error_x)<0.05)
 	   {
 	     v.angular.z = 0;
 	     v.linear.x = 0;
 	     pub.publish(v);
 	     break;
 	   }
 	  v.angular.z = error_z*3;
 	  v.linear.x = error_x*1;
 	  pub.publish(v);
 	  ros::spinOnce();
 	  usleep(200);
 }
 	return 1;
}

int main(int argc,char** argv)
{
 	ros::init(argc,argv,"turtle_pose");
 	turtlepose z;//生成一个对象z
 	ros::spin();
 	return 0;
}
