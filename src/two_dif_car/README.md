实验八:两轮差速小车

roslaunch two_dif_car two_dif_car.launch
rostopic echo /real_vel
rostopic echo /car_data
rostopic echo /odom
rqt(cmd_vel,car_cmd)
