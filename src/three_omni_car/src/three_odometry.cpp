//里程计
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>//里程计所需要的消息头文件
#include <tf/transform_broadcaster.h>//建立tf树
#include<ros/time.h>

double x = 0.0;//起始横坐标
double y = 0.0;//起始纵坐标
double theta = 0.0;//起始角度
//PS:涉及到x,y,z坐标都需要建立tf树
double vx = 0.0;//起始x方向速度
double vy = 0.0;//起始y方向速度

double vtheta = 0.0;//

ros::Time current_time, last_time;

void vel_callback(const geometry_msgs::Twist::ConstPtr& vel)//有ConstPtr就是vel->linear.x否则vel.linear.x
{
    //get current velocity
    vx = vel->linear.x;//小车x方向速度
    vy = vel->linear.y;//小车y方向速度
    vtheta = vel->angular.z;//小车角速度

    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();//小车跑过的时间差,转换为秒
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;//小车在x方向上的路程差
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;//小车在y方向上的路程差
    double delta_theta = vtheta * dt;//小车旋转过的角度

    x += delta_x;//当前小车x坐标
    y += delta_y;//当前小车y坐标
    theta += delta_theta;//当前小车角度
    last_time = current_time;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "three_odometry");
  ros::NodeHandle n;
/***********************************************************/
  //发布odom话题,通过在屏幕echo话题,将里程计打印到终端
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  //订阅小车发布的three_real_vel话题来获取小车的速度linear.x,linear.y,angular.z
  ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("three_real_vel",10, vel_callback);
/***********************************************************/
  tf::TransformBroadcaster odom_broadcaster;

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate loop_rate(10.0);
  while (ros::ok())
  {
      ros::spinOnce();//回调阻塞放在开头也是一样的
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

      //建立tf树,world->odom->base_footprint
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;

                //send the transform
                odom_broadcaster.sendTransform(odom_trans);

                //next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";

                //set the position
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                //set the velocity
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vtheta;

                //publish the message
                odom_pub.publish(odom);//发布odom
                loop_rate.sleep();
  }
}
