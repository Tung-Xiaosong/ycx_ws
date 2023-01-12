#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>//move_base自带的action
#include<actionlib/client/simple_action_client.h>//action客户端头文件
#include<navigation_patrol/SetGoals.h>//自定义消息服务
#include<std_srvs/Empty.h>
#include<tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;
AC *ac;
std::vector<move_base_msgs::MoveBaseGoal> goals;
bool start = false;//开始信号
int loop_times;//巡航次数
int myIndex = 0;//传入导航点的个数
int point_num=0;//传入导航点的个数
bool send_flag = false;//发送信号
std::vector<int> successed_goals;
std::vector<int> failed_goals;
void initial(){
    myIndex = 0;
    loop_times = 0;
    start = false;
    send_flag = false;
    successed_goals.clear();
    failed_goals.clear();
}
bool setGoalCb(navigation_patrol::SetGoals::Request &req, navigation_patrol::SetGoals::Response &res)//设置巡航目标点服务
{
    if(start){
        res.is_successful = false;
        std::cout<<"正在执行巡航任务，请先停止任务后再设置目标点"<<std::endl;
    }else{
        goals.clear();//清空巡航点
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";//坐标点基于map坐标系下
        for (size_t i = 0; i < req.pose_quat.size(); i++)//pose_quat装第一个导航点
        {
            goal.target_pose.pose = req.pose_quat[i];//pose_quat和pose都是geometry_msgs/Pose类型,在pose_quat里输入的值可以一一对应
            goals.push_back(goal);//将巡航点压入goals容器中
        }
        for (size_t i = 0; i < req.pose_yaw.size(); i++)//pose_yaw装第二个导航点
        {
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, req.pose_yaw[i].z);
            goal.target_pose.pose.position.x = req.pose_yaw[i].x;//pose_yaw是geometry_msgs/Point类型
            goal.target_pose.pose.position.y = req.pose_yaw[i].y;
            goal.target_pose.pose.orientation = quat;
            goals.push_back(goal);
        }
        res.is_successful = true;
        loop_times = req.loop;
        start = true;
        send_flag = true;
    }
    return true;
}

bool cancelCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){//取消导航命令
    initial();
    ac->cancelGoal();
    return true;
}

bool pauseCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)//暂停导航命令
{
  ac->cancelGoal();
  return true;
}

bool restartCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)//重启导航命令
{
  ac->sendGoal(goals[myIndex]);
  return true;
}

bool patrolCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)//从参数服务器传入导航点,并导航
{
  ros::NodeHandle n;
  goals.clear();//清空巡航点
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";//坐标点基于map坐标系下
  n.getParam("loop",loop_times);
  n.getParam("path_num",point_num);
  for(int i=1;i<=point_num;i++)
  {
  n.getParam("pose"+std::to_string(i)+"/position"+"/x",goal.target_pose.pose.position.x);
  n.getParam("pose"+std::to_string(i)+"/position"+"/y",goal.target_pose.pose.position.y);
  n.getParam("pose"+std::to_string(i)+"/position"+"/z",goal.target_pose.pose.position.z);
  n.getParam("pose"+std::to_string(i)+"/orientation"+"/x",goal.target_pose.pose.orientation.x);
  n.getParam("pose"+std::to_string(i)+"/orientation"+"/y",goal.target_pose.pose.orientation.y);
  n.getParam("pose"+std::to_string(i)+"/orientation"+"/z",goal.target_pose.pose.orientation.z);
  n.getParam("pose"+std::to_string(i)+"/orientation"+"/w",goal.target_pose.pose.orientation.w);
  goals.push_back(goal);
  }
  start = true;
  send_flag = true;
  return true;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "multi_nav_simple");
    ros::NodeHandle nh;
    ros::ServiceServer goal_server= nh.advertiseService("set_goals", setGoalCb);//设置巡航目标点服务,调用服务里设置巡航点
    ros::ServiceServer cancel_server= nh.advertiseService("cancel_nav", cancelCb);//取消巡航服务
    ros::ServiceServer pause_server= nh.advertiseService("pause_nav",pauseCb);//暂停巡航服务
    ros::ServiceServer restart_server=nh.advertiseService("restart_nav",restartCb);//重启服务
    ros::ServiceServer patrol_server=nh.advertiseService("patrol_nav",patrolCb);//通过参数服务器传入巡航点
    ac = new AC("move_base", true);
    ROS_INFO("wait for move base");
    if(!ac->waitForServer()){//move_base服务没有找到
        ROS_ERROR("move base action not find");
        return 0;
    }
    ROS_INFO("connected to move base");
    ros::Rate loop_sleep(2);
    initial();//初始化

    while (ros::ok())
    {
        ros::spinOnce();
        if(start){
            if(!goals.empty()){//goals容器里的值不为空
                if(send_flag){
                    ac->sendGoal(goals[myIndex]);//发送目标,导航中
                    send_flag = false;
                    sleep(1);
                }
                if(ac->getState()!=actionlib::SimpleClientGoalState::ACTIVE){//说明已经导航结束
                    if(ac->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)//导航成功
                        successed_goals.push_back(myIndex);//将导航成功的下标放入successed_goals容器中
                    else failed_goals.push_back(myIndex);//否则将导航失败的下标放入failed_goals容器中
                    std::cout<<"目前导航成功：";
                    for (size_t i = 0; i < successed_goals.size(); i++)//输出导航成功的点的下标
                    {
                        std::cout<<" "<<successed_goals[i];
                    }
                    std::cout<<std::endl;
                    std::cout<<"目前导航失败：";
                    for (size_t i = 0; i < failed_goals.size(); i++)//输出导航失败的点的下标
                    {
                        std::cout<<" "<<failed_goals[i];
                    }
                    std::cout<<std::endl;
                    send_flag = true;
                    myIndex++;
                }
                if(myIndex == goals.size()){//大小为5
                    loop_times--;
                    myIndex = 0;
                    if(loop_times == 0)
                        initial();
                }
            }
        }
        loop_sleep.sleep();
    }
    delete ac;
    return 0;
}

