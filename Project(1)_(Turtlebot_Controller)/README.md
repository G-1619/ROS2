<div align="center">
  <img src="https://github.com/user-attachments/assets/1d0998f2-2ba5-4d96-9b85-8461c93803ec"/>
</div>

# ROS2 Project(1) - "Turtlebot Controller"
PyQt 를 이용해 만든 GUI 로 Gazebo 상의 Turtlebot 을 제어한다.
<br/><br/><br/><br/><br/>

## [ Requirements ]
* ROS2 Humble
* PySide6
* TurtleBot3 Packages
* Python (3.10.12)
* PyQT
<br/><br/><br/>

## [ Usage ]
### 1. Download
> **my_world.launch.py** 은 */.../turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch 에 Download.*
```bash
#bash
git clone https://github.com/G-1619/ROS2/tree/main/Project(1)_(Turtlebot_Controller)
```
### 2. Launch turtlebot3_gazebo (with my_world)
```bash
#bash
ros2 launch turtlebot3_gazebo my_world.launch.py world:=/.../my_world.world
```
### 3. Run GUI
```bash
#bash
/bin/python3 /.../my_turtlebot_controller.py
```
<br/><br/><br/>

## [ Functional Details ]
### 1. Button 제어
<div>
  <img src="https://github.com/user-attachments/assets/75b93844-08c5-4038-8f56-e007fa7c1d04" width="400"/>
</div>

* 각 버튼으로 Turtlebot을 직접 제어할 수 있는 화면.  
* class MyTurtlebot3Controller(my_turtlebot_controller.py) 내에 정의된 Publisher node 에 의해 동작이 실행.
<br/>

### 2. 도형 주행
<div>
  <img src="https://github.com/user-attachments/assets/313768c9-14ee-4f3c-90d0-2d3cf2bdcefc" width="400"/>
</div>

* 도형(삼각형 / 사각형) 모양으로 자동 주행을 실행할 수 있는 화면.
* Action_Server(my_patrol_server.py) 와 Action_Client(my_patrol_client.py) 에 의해 Turtlebot 의 동작이 실행.
* 주행 중 장애물을 만나면 수행하던 동작을 정지함.
* 이동 중에는 선택 버튼들이 비활성화. 이를 위해 Action_Client 와 GUI 간의 signal 연결.
<br/>

### 3. 충돌 방지 주행
<div>
  <img src="https://github.com/user-attachments/assets/07a1245d-e5e2-46d5-9e5a-a71553ba2c04" width="400"/>
</div>

* Laser Sensor 를 활용해 장애물을 피해 자율주행을 실행할 수 있는 화면.
* class AutoDrive(my_auto_drive.py) 가 Publisher node 의 역할.
* 이동 중에는 "Start" 버튼이 비활성화. 이를 위해 AutoDrive node 와 GUI 간의 signal 연결.
* Turtlebot의 동작이 변화할 때마다 화면에 log 가 출력.
* 막다른 길에 끼임을 방지하기 위해 Turtlebot의 위치가 일정 시간동안 제자리일 때, 방향조정 수행.
<br/>

### 4. Main Button
* 각 화면 입력에 의해 동작하던 Turtlebot을 정지시킴.
* 새 입력에 대응할 준비를 하기 위함.
<br/><br/><br/>
