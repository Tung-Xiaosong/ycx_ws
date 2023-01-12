#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> //actionlib客户端头文件
#include "turtle_control_action/turtle_controlAction.h"//自定义action头文件
#include<actionlib/client/terminal_state.h>//包含goal可能用的一些状态
#include<turtle_control_action/Path.h>
#include<turtle_control_action/Point.h>
#include<cstdlib>
#include<cstdio>
#include<sstream>
void doneCb(const actionlib::SimpleClientGoalState& state, const turtle_control_action::turtle_controlResultConstPtr& result)
{//任务结束回调
    std::string ret = state.toString();
    if(ret == "SUCCEEDED")
    {
        ROS_INFO("task complete, request goal num = %d", result->point_count);
    }
    else
       ROS_INFO("Incomplete, -----Task response: %s -----", ret.c_str());
    ros::shutdown();//与Ctrl+c功能相似
}
void feedbackCb(const turtle_control_action::turtle_controlFeedbackConstPtr& feedback)
{//任务进行中间状态反馈
     ROS_INFO("moving, current goal: (%f, %f)", feedback->current_point.x, feedback->current_point.y);
}
void activeCb()
{//任务开始时回调
    ROS_INFO("Request received, start");
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"turtle_action_client");
  ros::NodeHandle n;
  actionlib::SimpleActionClient<turtle_control_action::turtle_controlAction> AC("turtle_action",true);//定义一个客户端

  ROS_INFO("Waiting for action server to start.");
  AC.waitForServer();
  ROS_INFO("Action server started,sending goal.");

//*******************通过param来传入坐标值********************************
  turtle_control_action::turtle_controlGoal goal;
  turtle_control_action::Point point;
  n.getParam("Times",goal.times);
  n.getParam("Path/count_num",goal.m_path.point_num);
  for(int i=1;i<=goal.m_path.point_num;i++)
  {
  n.getParam("Path/point"+std::to_string(i)+"/x",point.x);
  n.getParam("Path/point"+std::to_string(i)+"/y",point.y);
  goal.m_path.path_points.push_back(point);
  }
/*
  turtle_control_action::Point m_point1,m_point2,m_point3,m_point4,m_point5;
  turtle_control_action::turtle_controlGoal goal;
  m_point1.x =2;
  m_point1.y =2;
  m_point2.x =8;
  m_point2.y =2;
  m_point3.x =8;
  m_point3.y =8;
  m_point4.x =2;
  m_point4.y =8;
  m_point5.x =5;
  m_point5.y =5;
  goal.m_path.path_points.push_back(m_point1);
  goal.m_path.path_points.push_back(m_point2);
  goal.m_path.path_points.push_back(m_point3);
  goal.m_path.path_points.push_back(m_point4);
  goal.m_path.path_points.push_back(m_point5);
  goal.times=2;
  goal.m_path.point_num=5;
*/
  AC.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标
  ros::spin();//回调阻塞
  return 0;
}


