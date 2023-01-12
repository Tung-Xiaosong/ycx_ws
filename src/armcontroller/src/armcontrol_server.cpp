#include<ros/ros.h>
#include<serial/serial.h>
#include<armcontroller/goto_pos.h>//自定义服务goto_pos头文件
#include<armcontroller/relative_pos.h>//自定义服务relative_pos头文件
#include<armcontroller/pump.h>//自定义服务pump.srv头文件
#include<armcontroller/armcontrolMsg.h>//自定义消息头文件
#include<iostream>

serial::Serial _serial;//定义串口对象
std::string data="";
std::string command="";
float x,y,z;

class armcontrolserver
{
public:
  armcontrolserver();
private:
  ros::NodeHandle n;
  ros::ServiceServer goto_pos;//定义goto_pos服务!!!!!!!!!!!!!定义
  ros::ServiceServer relative_pos;//定义relative_pose服务
  ros::ServiceServer pump;//定义吸盘服务

  bool Goto(armcontroller::goto_pos::Request& req, armcontroller::goto_pos::Response& res);
  bool Relative(armcontroller::relative_pos::Request& req, armcontroller::relative_pos::Response& res);
  bool Pump(armcontroller::pump::Request& req, armcontroller::pump::Response& res);
 };
/******************类的构造函数*****************************/

armcontrolserver::armcontrolserver()//类的构造函数
{
  goto_pos = n.advertiseService("goto_pos",&armcontrolserver::Goto,this);//goto_pos服务的初始化!!!!!!!!!!!!!
  relative_pos = n.advertiseService("relative_pos",&armcontrolserver::Relative,this);//relative_pos服务的初始化!!!!!!!!!!
    pump = n.advertiseService("pump",&armcontrolserver::Pump,this);//pump服务的初始化!!!!!!!!!!
   std::cout<<"22222"<<std::endl;
}

/*************调用goto_pos服务*********************************/
bool armcontrolserver::Goto(armcontroller::goto_pos::Request& req, armcontroller::goto_pos::Response& res)//调用goto_pos服务
{
  std::cout<<"33333333"<<std::endl;
  char send_command[255];
  sprintf(send_command,"M2222 X%f Y%f Z%f\n",req.x,req.y,req.z);//从goto_pos.srv中传入值,确认是否可到达
  command = (std::string)send_command;//将命令转换成字符串型
  std::cout<<command<<std::endl;//输出命令到屏幕
  _serial.write(command.c_str());//写入命令
  usleep(200*1000);
  data = _serial.read(_serial.available());//从缓存区读入反馈的数据
  if(data.find("V0")<100)
  {
    std::cout<<"the point can not arrived!"<<std::endl;
    return -1;//change
  }
  else if(data.find("V1")<100)//表明坐标可到达
  {
    sprintf(send_command,"G0 X%f Y%f Z%f F10000\n",req.x,req.y,req.z);
    command = (std::string)send_command;
    std::cout<<command<<std::endl;
    _serial.write(command.c_str());//将移动命令写入
    usleep(50*1000);
    data = _serial.read(_serial.available());
    if(data.find("E22")<100||data.find("E26"))//change,表明坐标仍不可到达
    {
      std::cout<<"the point sure can not arrived!"<<std::endl;
      return -1;//change
    }
  }
  return 1;//注意最后要返回return 1否则屏幕上会显示错误
}

/*****************调用relative_pos服务**************************************/
bool armcontrolserver::Relative(armcontroller::relative_pos::Request& req,armcontroller::relative_pos::Response& res)
{
  char send_command[255];
  x+=req.x2;
  y+=req.y2;
  z+=req.z2;
  sprintf(send_command,"M2222 X%f Y%f Z%f\n",x,y,z);//从goto_pos.srv中传入值,确认是否可到达
  command = (std::string)send_command;//将命令转换成字符串型
  std::cout<<command<<std::endl;//输出命令到屏幕
  _serial.write(command.c_str());//写入命令
  usleep(200*1000);
  data = _serial.read(_serial.available());//从缓存区读入反馈的数据
  if(data.find("V0")<100)
  {
    std::cout<<"the point can not arrived!"<<std::endl;
    return -1;//change
  }
  else if(data.find("V1")<100)//表明坐标可到达
  {
    //sprintf(send_command,"G0 X%f Y%f Z%f F10000\n",req.x2,req.y2,req.z2);//#n G2204 X10 Y10 Z10 F1000\n
    sprintf(send_command,"G2204 X%f Y%f Z%f F10000\n",req.x2,req.y2,req.z2);
    command = (std::string)send_command;
    std::cout<<command<<std::endl;
    _serial.write(command.c_str());//将移动命令写入
    usleep(50*1000);
    data = _serial.read(_serial.available());
    if(data.find("E22")<100||data.find("E26"))//change,表明坐标仍不可到达
    {
      std::cout<<"the point sure can not arrived!"<<std::endl;
      return -1;//change
    }
  }
  return 1;
}

/************************自定义吸盘服务******************************************************/
bool armcontrolserver::Pump(armcontroller::pump::Request& req,armcontroller::pump::Response& res)
{
  char send_command[255];
  sprintf(send_command,"M2231 V%d\n",req.v);//从pump.srv中传入值
  command = (std::string)send_command;//将命令转换成字符串型
  std::cout<<command<<std::endl;
  _serial.write(command.c_str());
  usleep(100*1000);
  data = _serial.read(_serial.available());
  if(data.find("ok")<100)//这里需要清空缓存区数据,否则一直都是The pump is open!
  {
    ROS_INFO_STREAM("The pump is open!");
  }
  else
  {
    return -1;
  }
  return 1;
}

/************************MAIN函数**********************************************/
int main(int argc,char** argv)
{
  ros::init(argc,argv,"armcontroller");
	ros::NodeHandle n;
  ros::Publisher pub=n.advertise<armcontroller::armcontrolMsg>("arm_pos_info",10);//发布arm_pos_info话题
  _serial.setPort("/dev/ttyACM0");//设置串口
  _serial.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  _serial.setTimeout(to);
  _serial.open();//打开串口
  if(_serial.isOpen())
  {
    ros::Duration(3.5).sleep();				// wait 3.5s
    _serial.write("M2120 V0\r\n");			// stop report position(返回笛卡尔坐标)
    ros::Duration(0.1).sleep();				// wait 0.1s
    //_serial.write("M17\r\n");				// attach
    ros::Duration(0.1).sleep();				// wait 0.1s
    ROS_INFO_STREAM("Attach and wait for commands");
  }
  while (ros::ok())
  {
    std::string read_line;
    _serial.readline(read_line,65535,"\n");
    std::cout << read_line;
    if (read_line.find("@5 V1") < 1000) break;
    ros::Duration(0.05).sleep();				// wait 0.1s
/********************************************************///获得当前的坐标并输出到屏幕
  armcontroller::armcontrolMsg msg;
  command =(std::string)"P2220"+"\n";//获取当前坐标
  _serial.write(command.c_str());
  usleep(200*1000);
  std::string data="";
  data = _serial.read(_serial.available());//读入当前坐标
  std::cout<<data.c_str()<<std::endl;//输出
  std::string temp;
  int posex = -1;
  int posey = -1;
  int posez = -1;
  posex = data.find("X");
  posey = data.find("Y");
  posez = data.find("Z");

  temp = data.substr(posex+1,posey-posex);
  x = std::atof(temp.c_str());

  temp = data.substr(posey+1,posez-posey);
  y = std::atof(temp.c_str());

  temp = data.substr(posez+1,data.length()-posez-1);
  z = std::atof(temp.c_str());
  msg.x=x;
  msg.y=y;
  msg.z=z;
  pub.publish(msg);
  std::cout<<"the X,Y,Z is"<<x<<","<<y<<","<<z<<std::endl;//输出当前坐标的值
  ros::spinOnce();
  usleep(200*1000);
}
/*******************************************************/
  armcontrolserver armcontrolserver;//进入类
  ros::spin();
  return 0;
}
