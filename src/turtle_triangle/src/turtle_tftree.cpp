#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<turtlesim/Pose.h>
#include<geometry_msgs/Twist.h>/**/
using namespace std;
string turtle_name;

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
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));
  //发布小乌龟和世界坐标系之间的tf关系
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"turtle_tftree");//初始化节点
  if(argc!=2)//如果外界没有参数传递/*2*//*必须是2*/
  {
    ROS_ERROR("need turtle name as argument111");
    return -1;
  }
  turtle_name=argv[1];/*1*/
  ros::NodeHandle n;
  //订阅乌龟的pose信息
  ros::Subscriber sub=n.subscribe(turtle_name+"/pose",10,&poseCallback);
  //订阅到pose信息后跳转到回调函数poseCallback
  ros::spin();
  return 0;
}
/*
<!--打开turtle_formation_node节点创建carrot1和turtle1之间的tf关系-->
<node pkg="turtle_triangle" type="turtle_tftree_node" args="/turtle2" name="carrot1_formation_node"/>

<!--打开turtle_formation_node节点创建carrot2和turtle1之间的tf关系-->
<node pkg="turtle_triangle" type="turtle_tftree_node" args="/turtle3" name="carrot2_formation_node"/>
*/

/*
<!--打开turtle_formation_node节点创建carrot1和turtle1之间的tf关系-->
<node pkg="turtle_triangle" type="turtle_tftree_node" name="carrot1_formation_node"/>
*/
