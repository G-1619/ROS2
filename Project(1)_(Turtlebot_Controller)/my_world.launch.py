import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 기본 경로 설정
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # 로봇 모델 정보를 위해 필요함
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # 2. Launch Configuration 정의 (외부에서 변경 가능하게)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 기본 월드 경로를 사용자님의 월드로 설정 (절대 경로 추천)
    world_path = LaunchConfiguration('world', default='/home/사용자계정명/robot_ws/src/my_turtlebot_pkg/my_turtlebot_pkg/my_world.world')

    # 3. Launch Argument 선언 (터미널에서 world:=... 로 받을 수 있게 함)
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to world model file to load'
    )

    # 4. Gazebo 서버 실행 (설정한 world 경로 전달)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 5. Gazebo 클라이언트(GUI) 실행
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 6. Robot State Publisher (로봇의 관절 상태 등을 위해 필요)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # [주의] 월드 파일에 이미 터틀봇이 있다면 spawn_turtlebot_cmd는 추가하지 않습니다.
    # 만약 로봇이 안 나타난다면 아래 주석을 풀고 다시 추가하세요.
    # ld.add_action(spawn_turtlebot_cmd)

    ld = LaunchDescription()

    # 인자 추가
    ld.add_action(declare_world_arg)
    
    # 명령어 추가
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
