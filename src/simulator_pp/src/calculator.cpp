#include <pluginlib/class_loader.h>
#include <simulator_pp/simulator.h>
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculator_node");
    ros::NodeHandle nh("~");
    std::string method;
    nh.getParam("method", method);
    pluginlib::ClassLoader<my_simulator::Calculator> poly_loader("simulator_pp", "my_simulator::Calculator");
    boost::shared_ptr<my_simulator::Calculator> add
        = poly_loader.createInstance(method);
    add->init(2, 2);
    std::cout<<add->operate()<<std::endl;

  return 0;
}
