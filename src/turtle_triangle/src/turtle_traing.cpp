#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Spawn.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"turtle_traing");
  ros::NodeHandle n;
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle=n.serviceClient<turtlesim::Spawn>("spawn");
  ros::ServiceClient add_turtle2=n.serviceClient<turtlesim::Spawn>("spawn");//change
  turtlesim::Spawn srv;
  add_turtle.call(srv);//生成一个新的乌龟
  add_turtle2.call(srv);//生成一个新的乌龟//change
  //发布类型位速度类型的速度发布器
  ros::Publisher turtle_vel=n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);
  ros::Publisher turtle_vel2=n.advertise<geometry_msgs::Twist>("turtle3/cmd_vel",10);//change
  tf::TransformListener listener;//创建tf监听器
  tf::TransformListener listener2;//创建tf监听器//change
  ros::Rate rate(10.0);
  while (n.ok())
  {
    tf::StampedTransform transform;
    tf::StampedTransform transform2;
    try
    {
      //查找turtle2与carrot1的坐标转换
      listener.waitForTransform("/turtle2","/carrot1",ros::Time(0),ros::Duration(3.0));
      listener.lookupTransform("/turtle2","/carrot1",ros::Time(0),transform);
      //查找turtle3与carrot2的坐标转换//change
      listener2.waitForTransform("/turtle3","/carrot2",ros::Time(0),ros::Duration(3.0));
      listener2.lookupTransform("/turtle3","/carrot2",ros::Time(0),transform2);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;//速度消息
    geometry_msgs::Twist vel_msg2;//速度消息//change
    vel_msg.angular.z=10*atan2(transform.getOrigin().y(),transform.getOrigin().x());
    vel_msg.linear.x=5*sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));

    vel_msg2.angular.z=10*atan2(transform2.getOrigin().y(),transform2.getOrigin().x());//change
    vel_msg2.linear.x=5*sqrt(pow(transform2.getOrigin().x(),2)+pow(transform2.getOrigin().y(),2));

    turtle_vel.publish(vel_msg);//发布速度消息
    turtle_vel2.publish(vel_msg2);//发布速度消息//change
    rate.sleep();
  }
  return 0;
}
