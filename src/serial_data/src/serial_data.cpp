#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
          uint8_t data[33];
             uint8_t buffer[100];
    size_t m = 0;
     
    std::cout<<" the count is:"<<n<<std::endl;
    m = sp.read(buffer,65);
       for (int i = 0; i< m-1; i++)
    {
      if(buffer[i]==0x55)
      {
        if(buffer[i+1]==0x51)
        {
          for(int j = 0;j<33;j++)
          {
            data[j]=buffer[i+j];
            std::cout<<std::hex<<(data[j]&0xff)<<" ";
          }
          break;
        }
      }
    }
        std::cout<<std::endl;
    }
    //关闭串口
    sp.close();
    return 0;
    }
