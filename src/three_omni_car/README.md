实验九:三轮全向小车

roslaunch three_omni_car three_omni_car.launch
rostopic echo /three_real_vel
rostopic echo /car_data
rostopic echo /odom
rqt(three_cmd_vel,car_cmd)
