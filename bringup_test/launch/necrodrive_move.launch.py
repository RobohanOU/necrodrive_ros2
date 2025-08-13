"""Forward controller test."""
# -----------------------------NATSUROBOCON ROBOHAN-----------------------------
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
# Description: Used for testing. Here's how you use it:
#              1) Launch the necrodrive ros2_control stack with
#                 necrodrive.launch.py
#              2) Once the hardware interface has been loaded and running, run
#                 this launchfile. The odrive should move!
# Warning: For some reason, this doesn't work with JTC!! See the README
# Robohan 2025
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            'use_jtc',
            description='use joint trajectory controller instead of feed forward controller',
            default_value='true',
        ),
    ]

    use_jtc = LaunchConfiguration('use_jtc')

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare('necrodrive_ros2'),
            'bringup_test',
            'config',
            'trajectory_publisher.yaml' if IfCondition(use_jtc) else 'forward_position_pub.yaml',
        ]
    )

    return LaunchDescription(
        declared_arguments +
        [
            Node(
                package='ros2_controllers_test_nodes',
                executable='publisher_joint_trajectory_controller' if IfCondition(use_jtc)
                else 'publisher_forward_position_controller',
                name='publisher_joint_trajectory_controller' if IfCondition(use_jtc)
                else 'publisher_forward_position_controller',
                parameters=[position_goals],
                output='both',
            )
        ]
    )
