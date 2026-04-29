import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bringup = get_package_share_directory('my_robot_bringup')

    # 1ο - Simulation (Gazebo + RViz2)
    simulation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'my_robot_gazebo.launch.xml')
        )
    )

    # 2ο + 3ο - Περιμένει Gazebo, μετά ξεκινά navigation
    # και controller σε νέο terminal
    wait_and_launch = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "Waiting for Gazebo..." && '
            'until ros2 topic list | grep -q "/clock"; do sleep 1; done && '
            'echo "Gazebo ready! Starting navigation..." && '
            f'ros2 launch my_robot_bringup navigation.launch.xml & '
            'echo "Waiting for map..." && '
            'until ros2 topic info /map | grep -q "Publisher count: 1"; do sleep 1; done && '
            'echo "Map ready! Starting teleop..." && '
            'gnome-terminal -- bash -c "'
            'source /opt/ros/jazzy/setup.bash && '
            'ros2 run robot_controller_nav2 teleop_keyboard; exec bash"'
        ],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        wait_and_launch,
    ])