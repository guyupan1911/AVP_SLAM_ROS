sudo docker run -it --name="AVP" --privileged --net=host -e DISPLAY=$DISPLAY \
-v /tmp/.X11-unit:/tmp/.X11-unix --runtime=nvidia --device /dev/nvidia0 \
--device /dev/nvidia-uvm --device /dev/nvidia-uvm-tools \
--device /dev/nvidiactl \
-v $HOME/projects/AVP_SLAM_ROS:$HOME/catkin_ws/src/AVP_SLAM_ROS \
fdko11/ros:bionic-melodic-cartographer