/*ROS-CAN通信解析程序分析
open,打开can卡
initcan,对can卡初始化
start,启动can通道
接收receive和发送transmit
*/
/*
解析数据
发送数据
接收数据
*/
#include<ros/ros.h>
#include"can_control/controlcan.h"
#include<can_control/Enable.h>
#include<can_control/Speed.h>

union convert
{
  uint16_t objiect;
  uint8_t  start[2];
};

class Motor //Motor类
{
private:
  ros::NodeHandle n;
  ros::Publisher sendTopic_Speed;//创建发布者
  ros::Subscriber receiveTopic_Enable;
  ros::Subscriber receiveTopic_Speed;//创建接收者
  VCI_CAN_OBJ  Data[2];//Data[2]表示一个帧的数据结构--结构体。左右电机
  VCI_INIT_CONFIG Can_Config;//初始化CAN的配置,初始化之前，要先填好这个结构体变量。
  uint16_t node_adress;
  ros::Rate rate;
  uint callNum;
  can_control::Speed speed_value;
  convert trans;
  bool receive_flag;
public:
  bool Flag;
  Motor();  //构造函数
  ~Motor();
  void Execute();
  void Init();
  void Clearn_Struct();
  void Speed_Callback(const can_control::SpeedConstPtr & speed);
  void Enable_Callback(const can_control::EnableConstPtr & enable);
};

int main(int argc,char ** argv) //主函数
{
  ros::init(argc, argv, "motor_control_can_node");
  Motor motor;//类
  motor.Init();//类初始化
  if(motor.Flag)
    return 0;
  motor.Execute();
  return 0;
}

Motor::Motor():rate(100)  //构造函数
{
  sendTopic_Speed=n.advertise<can_control::Speed>("/can_motor_speed",10); //发布can_motor_speed话题
  receiveTopic_Enable=n.subscribe("/can_Enable",10,&Motor::Enable_Callback,this); 
  receiveTopic_Speed=n.subscribe("/can_cmd_vel",10,&Motor::Speed_Callback,this);
  Flag=false;
  receive_flag=true;
  Can_Config.AccCode=0x00000000;
  Can_Config.AccMask=0xFFFFFFFF;
  Can_Config.Filter=1;
  Can_Config.Timing0=0x00;
  Can_Config.Timing1=0x14;
  Can_Config.Mode=0;
  if(1!=VCI_OpenDevice(4,0,0))//打开设备
  {
    ROS_WARN("Device Open Is Failed!");
    Flag=true;
    return;
  }
  if(1!=VCI_InitCAN(4,0,0,&Can_Config))//此函数用以初始化指定的CAN通道。有多个CAN通道时，需要多次调用。
  {
    ROS_WARN("Can Init Is Failed!");
    Flag=true;
    return;
  }
  ROS_WARN("Can Init Is Success!");
  if(1!=VCI_StartCAN(4,0,0))//用以启动CAN卡的某一个CAN通道。有多个CAN通道时，需要多次调用
  {
    ROS_WARN("Can Start Is Failed!");
    Flag=true;
    return;
  }
  ROS_WARN("Can Start Is Success!");
}

void Motor::Enable_Callback(const can_control::EnableConstPtr &enable)  //发送数据
{
  Clearn_Struct();
  Data[0].ID=0x00+node_adress;  //0x00+节点地址
  Data[0].DataLen=8;            //数据长度为8
  Data[0].Data[0]=0x00;
  if(enable->Enable)
    Data[0].Data[2]=0x0a;
  else
    Data[0].Data[2]=0x01;
  VCI_Transmit(4,0,0,Data,1); //发送数据
}

void Motor::Clearn_Struct() //清空数据结构体
{
  for (int i=0;i<2;i++)
  {
    Data[i].TimeStamp=0;
    Data[i].TimeFlag=0;
    Data[i].RemoteFlag=0;
    Data[i].ExternFlag=0;
    Data[i].SendType=1;
    Data[i].DataLen=0;
    for (int j=0;j<8;j++)
      Data[i].Data[j]=0;
  }
}

void Motor::Speed_Callback(const can_control::SpeedConstPtr &speed)
{
  Clearn_Struct();
  Data[0].ID=0x00+node_adress;
  Data[0].DataLen=8;
  Data[0].Data[0]=0x03;
  trans.objiect=static_cast<uint16_t>(speed->speed[0])*27;
  Data[0].Data[1]=trans.start[1];
  Data[0].Data[2]=trans.start[0];
  trans.objiect=static_cast<uint16_t>(speed->speed[1])*27;
  Data[0].Data[3]=trans.start[1];
  Data[0].Data[4]=trans.start[0];
  VCI_Transmit(4,0,0,Data,1);
}

void Motor::Init()
{
  if(Flag) return ;
  Clearn_Struct();
  Data[0].ID=0x00;
  Data[0].DataLen=8;
  Data[0].Data[0]=0x00;
  Data[0].Data[1]=0x00;
  Data[0].Data[2]=0x0a;
  receive_flag=true;
  VCI_ClearBuffer(4,0,0);
  while(receive_flag&&ros::ok())
  {
    callNum=VCI_Transmit(4,0,0,Data,1);
    if(-1U==callNum)
    {
      ROS_WARN("Transmit Is Failed!");
      Flag=true;
      return;
    }
    callNum=VCI_GetReceiveNum(4,0,0);
    if(callNum<1)
    {
      usleep(10*1000);
      continue;
    }
    callNum=VCI_Receive(4,0,0,&Data[1],1,0);
    if(-1U==callNum)
    {
      ROS_WARN("Receive Is Failed!");
      Flag=true;
      return;
    }
    if(0x00==Data[1].Data[0]&&1==Data[1].ID/0x80)
    {
      receive_flag=false;
      node_adress=static_cast<uint16_t>(Data[1].ID)-0x80;
      ROS_INFO("Enable is true;node_adress is:%d",node_adress);
    }
  }
}

void Motor::Execute()
{
  while (ros::ok())
  {
    ros::spinOnce();
    Clearn_Struct();
    callNum=VCI_GetReceiveNum(4,0,0);
    if(callNum<1&&ros::ok())//用以获取指定CAN通道的接收缓冲区中，接收到但尚未被读取的帧数量。
    {
      rate.sleep();
      continue;
    }
    callNum=VCI_Receive(4,0,0,&Data[1],1,0);
    if(-1U==callNum)
    {
      ROS_WARN("Receive Is Failed!");
      return;
    }
    if(0x00==Data[1].Data[0]&&(0X280+node_adress)==Data[1].ID)
    {
      trans.start[1]=Data[1].Data[1];
      trans.start[0]=Data[1].Data[2];
      speed_value.speed[0]=trans.objiect/27;
      trans.start[1]=Data[1].Data[3];
      trans.start[0]=Data[1].Data[4];
      speed_value.speed[1]=trans.objiect/27;
      sendTopic_Speed.publish(speed_value);
    }
    rate.sleep();
  }
}

Motor::~Motor()
{
  if(1!=VCI_CloseDevice(4,0))
    ROS_WARN("Device Close Is Failed!");
}
