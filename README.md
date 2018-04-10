# RobotComp

export IPBerry=192.168.0.141
ssh pi@$IPBerry
senha: t0rt1g4
screen
Ctrl A, C     x3

Ctrl A, "

(1) roslaunch turtlebot3_bringup turtlebot3_core.launch

(2) roslaunch turtlebot3_bringup turtlebot3_lidar.launch

(3) roslaunch raspicam_node camerav2_640x480_30fps.launch

Ctrl A, D

FECHAR A RASPBERRY
sudo shutdown -P now

NO PC

PRIMEIRO TERMINAL

export IPBerry=192.168.0.141
export ROS_MASTER_URI="http://"$IPBerry":11311"
export ROS_IP=`hostname -I`
export TURTLEBOT3_MODEL=burger
roscore

SEGUNDO TERMINAL

export IPBerry=192.168.0.141
export ROS_MASTER_URI="http://"$IPBerry":11311"
export ROS_IP=`hostname -I`
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_remote.launch

TERCEIRO  TERMINAL

export IPBerry=192.168.0.141
export ROS_MASTER_URI="http://"$IPBerry":11311"
export ROS_IP=`hostname -I`
export TURTLEBOT3_MODEL=burger
rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz

QUARTO TERMINAL

export IPBerry=192.168.0.141
export ROS_MASTER_URI="http://"$IPBerry":11311"
export ROS_IP=`hostname -I`
export TURTLEBOT3_MODEL=burger
rosrun rqt_reconfigure rqt_reconfigure

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


NO SIMULADOR

roslaunch turtlebot3_gazebo turtlebot3_world.launch

rosparam set cv_camera/device_id 0
rosparam set cv_camera/cv_cap_prop_frame_width 640
rosparam set cv_camera/cv_cap_prop_frame_height  480
rosrun cv_camera cv_camera_node
