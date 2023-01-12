#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
using namespace std;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"turtle_formation");//初始化节点
  ros::NodeHandle n;
  tf::TransformBroadcaster br;//tf广播器
    tf::TransformBroadcaster br2;/*********************/
  tf::Transform transform;//定义一个tf关系
   tf::Transform transform2;//定义一个tf关系/********************/
  ros::Rate rate(10.0);
  while(n.ok())
  {
    transform.setOrigin(tf::Vector3(-1.73,1.0,0.0));//设置平移变换
    transform.setRotation(tf::Quaternion(0,0,0,1));//设置旋转变换
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"turtle1","carrot1"));//发布carrot1和turtle1坐标系之间的tf关系

    transform2.setOrigin(tf::Vector3(-1.73,-1.0,0.0));//设置平移变换
    transform2.setRotation(tf::Quaternion(0,0,0,1));//设置旋转变换
    br.sendTransform(tf::StampedTransform(transform2,ros::Time::now(),"turtle1","carrot2"));//发布carrot2和turtle2坐标系之间的tf关系
    rate.sleep();
  }
  return 0;
}

/*
int main(int argc,char **argv)
{
  ros::init(argc,argv,"turtle_formation");//初始化节点
  if(argc!=1)//如果外界没有参数传递2
  {
    ROS_ERROR("need turtle name as argument222");
    return -1;
  }
  turtle_name=argv[1];
  ros::NodeHandle n;
  //订阅乌龟的pose信息
  ros::Subscriber sub=n.subscribe(turtle_name+"/pose",10,&poseCallback);
  //订阅到pose信息后跳转到回调函数poseCallback
  ros::spin();
  return 0;
}

void poseCallback(const turtlesim::PoseConstPtr&msg)
{
  //tf广播器
  static tf::TransformBroadcaster br;
  //根据乌龟当前位姿，设置相对于世界坐标系的坐标变换
  tf::Transform transform;//定义一个tf关系
  transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));//设置平移变换
  tf::Quaternion q;//定义四元数
  q.setRPY(0,0,msg->theta);//用rpy初始化四元数
  transform.setRotation(q);//设置旋转变换
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"turtle1",turtle_name));//turtle1
  //发布carrot和turtle1坐标系之间的tf关系
}
*/
