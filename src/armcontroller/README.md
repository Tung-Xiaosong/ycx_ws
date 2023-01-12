实验七:编写客户端和服务端程序实现机械臂移动

roscore
sudo chmod 666 /dev/ttyACM0
rosrun armcontroller armcontrol_client_node
rosrun armcontroller armcontrol_server_node
rosservice call /...
