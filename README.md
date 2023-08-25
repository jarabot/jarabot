# jarabot
## jarabot

```bash
# ~/.bashrc에 source /opt/ros/humble/setup.bash 확인
sudo apt purge brltty
sudo apt install ros-humble-serial-driver \
ros-humble-teleop-twist-keyboard \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-cartographer \
ros-humble-cartographer-ros \
udev \
ufw

echo -e "\nexport ROS_DOMAIN_ID=<원하는 ID>" >> ~/.bashrc

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

ros2 launch jarabot_node bringup.launch.py # jarabot 터미널
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel # jarabot 터미널
```

## jarabot cartographer 실행 (지도 생성)
```bash
ros2 launch jarabot_node cartographer.launch.py # jarabot 터미널
rviz2 -d ~/ros2_ws/src/jarabot/jarabot_cartographer/rviz/jarabot_cartographer.rviz #local pc 에서 실행
```

## jarabot navigation2 실행
```bash
ros2 launch jarabot_node navigate.launch.py map:=$HOME/map.yaml # 절대경로 사용 필수
rviz2 -d ~/ros2_ws/src/jarabot/jarabot_navigation2/rviz/jarabot_navigation2.rviz
```

