#include<ros/ros.h>
#include<serial/serial.h>
#include<sensor_msgs/Imu.h>
#include<tf/tf.h>
int main(int argc, char **argv)
{
 ros::init(argc,argv,"imu_data_node");
 ros::NodeHandle n;

 serial::Serial imu_sp;
 serial::Timeout to = serial::Timeout::simpleTimeout(100);
 ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_data",10);

 imu_sp.setPort("/dev/ttyUSB0");
 imu_sp.setBaudrate(115200);
 imu_sp.setTimeout(to);

 imu_sp.open();
 if(imu_sp.isOpen())
 {
    std::cout<<"/dev/ttyUSB0 is opened"<<std::endl;
 }
 else
 {
   return 1;
 }
 //imu_sp.flushInput();
 ros::Rate loop(100);
 while (ros::ok())
  {
    uint8_t buffer[100];
    uint8_t data[33];
    size_t m = 0;
    size_t n = imu_sp.available();
    std::cout<<" the count is:"<<n<<std::endl;
    m = imu_sp.read(buffer,65);
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
    uint8_t DataH,DataL;
    DataH = data[3];
    DataL = data[2];
    float ax;
    ax = ((short)(((short)DataH)*256+DataL))/32768.f*16*9.8;

    DataH = data[5];
    DataL = data[4];
    float ay;
    ay = ((short)(((short)DataH)*256+DataL))/32768.f*16*9.8;

    DataH = data[7];
    DataL = data[6];
    float az;
    az = ((short)(((short)DataH)*256+DataL))/32768.f*16*9.8;
    ROS_INFO("ax:%f,ay:%f,az:%f",ax,ay,az);

    DataH = data[14];
    DataL = data[13];
    float wx;
    wx = ((short)(((short)DataH)*256+DataL))/32768.f*2000*3.1415926/180;

    DataH = data[16];
    DataL = data[15];
    float wy;
    wy = ((short)(((short)DataH)*256+DataL))/32768.f*2000*3.1415926/180;

    DataH = data[18];
    DataL = data[17];
    float wz;
    wz = ((short)(((short)DataH)*256+DataL))/32768.f*2000*3.1415926/180;
    ROS_INFO("wx:%f,wy:%f,wz:%f",wx,wy,wz);

    DataH = data[25];
    DataL = data[24];
    float Roll;
    Roll = ((short)(((short)DataH)*256+DataL))/32768.f*3.1415926;

    DataH = data[27];
    DataL = data[26];
    float Pitch;
    Pitch = ((short)(((short)DataH)*256+DataL))/32768.f*3.1415926;

    DataH = data[29];
    DataL = data[28];
    float Yaw;
    Yaw = ((short)(((short)DataH)*256+DataL))/32768.f*3.1415926;
    ROS_INFO("Roll:%f,Pitch:%f,Yaw:%f",Roll,Pitch,Yaw);

    sensor_msgs::Imu m_imu;
    m_imu.linear_acceleration.x = ax;
    m_imu.linear_acceleration.y = ay;
    m_imu.linear_acceleration.z = az;

    m_imu.angular_velocity.x = wx;
    m_imu.angular_velocity.y = wy;
    m_imu.angular_velocity.z = wz;

    tf::Quaternion q;
    q.setRPY(Roll,Pitch,Yaw);
    m_imu.orientation.w = q.w();
    m_imu.orientation.x = q.x();
    m_imu.orientation.y = q.y();
    m_imu.orientation.z = q.z();
    pub.publish(m_imu);
    loop.sleep();
  }
  imu_sp.close();
  return 0;
}

/*#include<ros/ros.h>
#include<serial/serial.h>
#include<sensor_msgs/Imu.h>
#include<tf/tf.h>
#include<iostream>
using namespace std;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"imu_data");
  ros::NodeHandle n;
  serial::Serial sp;
  serial::Timeout to=serial::Timeout::simpleTimeout(100);
  ros::Publisher pub=n.advertise<sensor_msgs::Imu>("imu_data",10);
  sp.setPort("/dev/ttyUSB0");
  sp.setBaudrate(115200);
  sp.setTimeout(to);
  try {
    sp.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }
  if(sp.isOpen())
  {
    ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
  }
  else {
    return -1;
  }
  ros::Rate loop_rate(100);
  /**********************************/
/*
  while(ros::ok())
  {
    size_t n=sp.available();
    uint8_t imu_data[33];
    uint8_t buffer[1024];
    double g=9.8;
    double m=0;
    cout<<"The count is:"<<n<<endl;
    if(n!=0)
    {
      m=sp.read(buffer,65);
      for (int i=0;i<m;i++)
      {
        if(buffer[i]==0x55)
          if(buffer[i+1]==0x52)
          {
            for (int j=0;i<33;j++)
            {
              imu_data[j]=buffer[i+j];
            }
            break;
          }
      }
    }
    //提取角速度
    double wx=((imu_data[3]<<8|imu_data[2])/32768*2000);
    double wy=((imu_data[5]<<8|imu_data[4])/32768*2000);
    double wz=((imu_data[7]<<8|imu_data[6])/32768*2000);
    //提取角度
    double Roll=((imu_data[14]<<8)|imu_data[13])/32768*180;
    double Pitch=((imu_data[16]<<8)|imu_data[15])/32768*180;
    double Yaw=((imu_data[18]<<8)|imu_data[17])/32768*180;
    //提取加速度
    double ax=((imu_data[25]<<8)|imu_data[24])/32768*16*g;
    double ay=((imu_data[27]<<8)|imu_data[26])/32768*16*g;
    double az=((imu_data[29]<<8)|imu_data[28])/32768*16*g;

    cout<<endl<<"角速度:"<<endl<<"wx="<<wx<<endl<<"wy="<<wy<<endl<<"wz="<<wz<<endl;
    cout<<"角度:"<<endl<<"Roll="<<Roll<<endl<<"Pitch="<<Pitch<<endl<<"Yaw="<<Yaw<<endl;
    cout<<"加速度:"<<endl<<"ax="<<ax<<endl<<"ay="<<ay<<endl<<"az="<<az<<endl<<"********";
    loop_rate.sleep();
  }
  sp.close();
  return 0;
}
*/
