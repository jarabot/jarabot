# jarabot
## jarabot

~/.bashrc에 source /opt/ros/humble/setup.bash 확인
```bash
sudo apt purge brltty
sudo apt install ros-humble-serial-driver \
ros-humble-teleop-twist-keyboard \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-cartographer \
ros-humble-cartographer-ros \
ros-humble-rmw-cyclonedds-cpp \
udev \
ufw

echo -e "\nexport ROS_DOMAIN_ID=<원하는 ID>" >> ~/.bashrc
echo -e "\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/jarabot/jarabot.git
cd ~/ros2_ws
colcon build --symlink-install

sudo cp ~/ros2_ws/src/jarabot/jarabot_node/rule/99-jarabot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo ufw enable
sudo ufw allow 1:65535/tcp
sudo ufw allow 1:65535/udp
sudo ufw status # tcp, udf rule 추가 확인

sudo reboot  # 리부팅하기

ls /dev/     # /dev/mydriver, /dev/mylidar 확인

source ~/ros2_ws/install/setup.bash

ros2 launch jarabot_node bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel
```

## jarabot cartographer 실행 (지도 생성)
```bash
ros2 launch jarabot_node cartographer.launch.py
rviz2 -d ~/projects/temp/jarabot/jarabot_cartographer/rviz/jarabot_cartographer.rviz
```

## jarabot navigation2 실행
```bash
ros2 launch jarabot_node navigate.launch.py map:=$HOME/map.yaml # 절대경로 사용!
rviz2 -d ~/projects/temp/jarabot/jarabot_navigation2/rviz/jarabot_navigation2.rviz
```

