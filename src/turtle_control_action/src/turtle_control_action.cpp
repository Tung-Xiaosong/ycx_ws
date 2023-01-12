#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "turtle2pose/position.h"
#include "turtle_control_action/turtle_controlAction.h"
#include<turtle_control_action/Path.h>

#include<turtle_control_action/Point.h>
using namespace std;
class turtle_ac
{
   public:
        turtle_ac(std::string name);//构造函数
        void turtle_action_move(const turtle_control_action::turtle_controlGoalConstPtr &goal);//action服务回调函数
   private:
        ros::NodeHandle n;//定义句柄
        actionlib::SimpleActionServer<turtle_control_action::turtle_controlAction> as;//定义action服务器
        ros::ServiceClient position_client;//小乌龟位置控制客户端
        turtle2pose::position position_data;//小乌龟位置控制数据
        turtle_control_action::turtle_controlFeedback feedback;//action反馈变量
        turtle_control_action::turtle_controlResult result;//action执行结果变量
};
turtle_ac::turtle_ac(std::string name):as(n,name,boost::bind(&turtle_ac::turtle_action_move,this,_1),false)//初始化ac
{
   position_client = n.serviceClient<turtle2pose::position>("pose_srv");//小乌龟控制客户端初始化
   as.start();//开启服务
}

void turtle_ac::turtle_action_move(const turtle_control_action::turtle_controlGoalConstPtr &goal)
{
  int count = 0;//用于记录小乌龟走过了多少个点；
  for(int i=0;i<goal->times;i++)
    {
      for(int j=0;j<goal->m_path.point_num;j++)
         {
         
            if(as.isPreemptRequested()||!ros::ok())//preempt 是否取消正在执行的动作
            {
                ROS_INFO("action Preempted");
                as.setPreempted();
                return;
            }            
            position_data.request.x = goal->m_path.path_points[j].x;
            position_data.request.y = goal->m_path.path_points[j].y;

            feedback.current_point.x = position_data.request.x;
            feedback.current_point.y = position_data.request.y;//反馈当前要执行的点
            as.publishFeedback(feedback);//发布当前的位置状态

            position_client.call(position_data);//调用服务控制小乌龟走向目标点
            count++;
         }
    }

    result.point_count = count;
    ROS_INFO("action Succeeded");
    as.setSucceeded(result);//action成功执行，返回结果
    return;
}
int main(int argc,char** argv)
{  
    ros::init(argc,argv,"turtle_control_action");
  	turtle_ac m_turtle("turtle_action");
  	ros::spin();
  	return 0;
}
