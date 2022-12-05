# robotAI
This is the program for Tsukuba Robot Contest 2022

# 依存パッケージのインストール
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

## arduinoインストール
sudo apt-get update  
sudo apt-get install arduino arduino-core

## rosserialのインストール
sudo apt-get update  
sudo apt-get install ros-melodic-rosserial-arduino  
sudo apt-get install ros-melodic-rosserial  
sudo usermod -a -G dialout YOUR_USER  

## roslibのインストール
cd sketchbook/libraries  
rm -rf ros_lib  
rosrun rosserial_arduino make_libraries.py .  

## urg_nodeのインストール
sudo apt-get install ros-melodic-urg-node

## catkin buildのインストール
sudo apt install python-catkin-tools

## USBシリアル接続時に自動で書込権限を付与する  
sudo vi /lib/udev/rules.d/50-udev-default.rules  
(Before)  
KERNEL=="tty[A-Z]*[0-9]|pppox[0-9]*|ircomm[0-9]*|noz[0-9]*|rfcomm[0-9]*", GROUP="dialout"  
(After)  
KERNEL=="tty[A-Z]*[0-9]|pppox[0-9]*|ircomm[0-9]*|noz[0-9]*|rfcomm[0-9]*", GROUP="dialout", MODE="0777"  # or 0666

# Inverse kinematics model of independent 3-wheel steering
θ is a constant
![Inverse kinematics model](images/Inverse_kinematics_model.jpg)

# Forward kinematics model
Using pseudo-inverse matrix  
![forward kinematics model](images/forward_kinematics_model.jpg)

# 手動操縦  
ステアを並進、回転移動させる（無限回転）  
「5」「6」同時押しでステア角を0に設定する  
roslaunch joy_control control4.launch  
![joy_control](images/joy_control.png)

# 自律走行  
roslaunch navi navi.launch  
![self_navigation](images/Self-navigation.png)
