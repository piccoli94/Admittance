# run the netft ROS node for ATI readings
roslaunch netft_rdt_driver ft_sensor.launch

# remove ATI bias
rosservice call /ft_sensor/bias_cmd "cmd: 'bias'"
