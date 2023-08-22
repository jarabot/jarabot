# jarabot
## jarabot
```bash
sudo apt purge brltty
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/babakhani/rplidar_ros2.git
git clone https://github.com/jebiio/jarabot.git
cd ~/ros2_ws
colcon build

sudo cp ~/ros2_ws/src/jarabot/jarabot_node/rule/99-jarabot.rules /etc/udev/rules.d/

sudo apt install udev

sudo udevadm control --reload-rules
sudo udevadm trigger

ls /dev/     # /dev/ttyUSB0 확인

source ~/ros2_ws/install/setup.bash

ros2 launch jarabot_node test.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel
```

## jarabot cartographer 실행 (지도 생성)
```bash
ros2 launch jarabot_cartographer cartographer.launch.py
```

## jarabot navigation2 실행
```bash
ros2 launch jarabot_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

