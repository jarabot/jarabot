# Gazebo Simulation SLAM
* 목차
  1. Simulation World 실행하기
  2. SLAM node 실행하기
  3. Teleop로 조정하기
  4. map 저장
  

* 장점
  * 여러 가상 환경 선택 가능
  * 실제 robot이 없어도 가능

## 1. Simulation World 실행하기
* 새 터미널에서 실행
```bash
ros2 launch jarabot_gazebo jarabot_world.launch.py
```

## 2. SLAM node 실행하기
* 새 터미널에서 실행
```bash
ros2 launch jarabot_cartographer cartographer.launch.py use_sim_time:=True
```

## 3. Teleop Node 실행하기
```bash
ros2 run jarabot_teleop teleop_keyboard
```

## 4. map 저장하기
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
* map.pgm 파일 저장

