"""
Launch file for testing necrodrive functionality.

here are the assumptions made:
can interface: can0
axis node id: 0x0

this can be modified in the sdf file

"""
# -----------------------------NATSUROBOCON ROBOHAN----------------------------
#    🮕             🮘  🭦🮄🮄🮄🮄🮃🮃🮃🮃🮃🮃🮂🮂🮂🮂🭛   🮘 🭦🭏▂▂▂▂▃▃▃█🭍▄▅▅▆▆   🭔🭀  🮘       🮕
#         🮘     🮕    🮕 ⎞ 🭦🮄🮄🮄║🮃🮃🮂🮂🮂 ⎛🮘           🭅🭛  🭥🭔🭍🭑▂▁▂🭐 🭭     🮕
#             🮘    🮕   ║ 🭦🮄🮄🮃║🮃🮃🮂🮂🭛 ║  🮕   🮘 ║🮀🮀🮀🮀🮀🮀🮀🮀🮀🮀▌ 🮂🮂   🮕              🮕
#        🮕          🮘 🭵🭱  ║  ║   ║ 🭵🭱        ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬍          🮘
#             🮕       🭴🭰 🭋▂▂▃║▃▄▄🭛  ║  🮘     ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋   🮕        🮕
#      🮕        🮘   🮕 ║     🭵🭱      🭴🭰    🮕  🭦🮄🮄🮄🮄🮄🮃🮃🮃🮃🮂🮂🮂🮂🮂🭌  🮘     🮕
#                  🮘  ║     ║▁▂▂▃🭎   ║🭋🭡 🮘  🭅🭀 🭃🭌  🭃🭌  🭃🭌   ▊
#        🮕      🮕    ⎠⎠ 🭦🮅🮅🮄🮄🮂🮂   🭦   V     🭒🭡 🭦🭡  🭦🭡  🭦🭡 🭦🭩🭡     🮘    🮕
# -----------------------------necrodrive.launch.py----------------------------
# Maintainer: Oz
# Description:  Necrodrive ros2_control hardware interface launchfile
#               Derived from ros2_control example_1
#               See the README for usage instructions
# Robohan 2025
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    """Generate and return launch description."""
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_jtc',
            default_value='true',
            description='Use JTC or feedforward controller'
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration('gui')
    use_jtc = LaunchConfiguration('use_jtc')  # jtc = joint trajectory control

    this_package = get_package_share_directory('necrodrive_ros2')

    # xacro path
    robot_description_path = os.path.join(
        this_package,
        'bringup_test',
        'sdf',
        'robot.sdf.xacro',
    )
    # turn xacro into sdf/urdf
    robot_description_content = xacro.process_file(robot_description_path).toprettyxml(indent='   ')
    robot_description = {'robot_description': robot_description_content}

    # controller yaml config filepath
    robot_controllers = PathJoinSubstitution(
        [
            this_package,
            'bringup_test',
            'config',
            'jtc.yaml' if IfCondition(use_jtc) else 'controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [this_package, 'bringup_test', 'rviz', 'necrodrive_test.rviz']
    )

    # controller manager node. Manages the controller
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    # publishes the robot's description for rviz and stuff
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
    )

    # publishes the state of the controlled joint, for rviz and stuff
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # responsible for spawning the actual controller
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_position_controller' if IfCondition(use_jtc) else
                'forward_position_controller',
            '--param-file', robot_controllers],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
