# Gazebo Simulation Navigation
* 목차
  1. Simulation World 실행하기
  2. Navigation node 실행하기
  3. 초기 Pose 추정하기
  4. Navigation Goal 설정하기
  

##  1. Simulation World 실행하기
```bash
ros2 launch jarabot_gazebo jarabot_world.launch.py
```

##  2. Navigation node 실행하기
```bash
$ ros2 launch jarabot_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

##  3. 초기 Pose 추정하기
* '2D Pose Estimate' 버튼 눌러서 위치와 방향을 지정

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/ros2/tb3_navigation2_rviz_01.png)

* teleop node 실행하기
```bash
ros2 run jarabot_teleop teleop_keyboard
```

##  4. Navigation Goal 설정하기
* 'Navigation2 Goal' 버튼 클릭하여 목표지점 설정
* 