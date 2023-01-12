麦克纳姆轮
roslaunch four_omni_car four_omni_car.launch
rostopic echo /four_real_vel
rostopic echo /car_data
rostopic echo /odom
rqt(four_cmd_vel,car_cmd)
