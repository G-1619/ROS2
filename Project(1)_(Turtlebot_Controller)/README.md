<div align="center">
  <img src="https://github.com/user-attachments/assets/f7e1dd72-5096-4283-b58a-f31c08e4c06f"/>
</div>


# ROS2 Project(1) - "Turtlebot Controller"
PyQt 를 이용해 만든 GUI 로 Gazebo 상의 Turtlebot 을 제어한다.
<br/><br/><br/><br/><br/>

## [ Requirements ]
* ROS2 Humble
* PySide6
* TurtleBot3 Packages
* Python (3.10.12)
<br/><br/><br/>

## [ Usage ]
### 1. Download
**my_world.launch.py** 은 */.../turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch 에 Download.*
```bash
#bash
git clone https://github.com/G-1619/ROS2/tree/main/Project(1)_(Turtlebot_Controller)
```
<br/>

### 2. Launch turtlebot3_gazebo (with my_world)
```bash
#bash
ros2 launch turtlebot3_gazebo my_world.launch.py world:=/.../my_world.world
```
<br/>

### 3. Run GUI
```bash
#bash
/bin/python3 /.../my_turtlebot_controller.py
```
<br/><br/><br/>
