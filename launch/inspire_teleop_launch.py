from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    # left_port_arg = DeclareLaunchArgument(
    #     'left_port',
    #     default_value='/dev/ttyUSB0',
    #     description='Serial port for the left device'
    # )
    # right_port_arg = DeclareLaunchArgument(
    #     'right_port',
    #     default_value='/dev/ttyUSB1',
    #     description='Serial port for the right device'
    # )
    finger_config_path_arg = DeclareLaunchArgument(
        'finger_config_path',
        default_value='./finger_config.yaml',
        description='Path to the finger configuration file'
    )
    robot_config_path_arg = DeclareLaunchArgument(
        'robot_config_path',
        default_value='./robot_config.yaml',
        description='Path to the robot configuration file'
    )
    left_calib_path_arg = DeclareLaunchArgument(
        'left_calib_path',
        default_value='./model/left_calib.yaml',
        description='Path to the left calibration file'
    )
    right_calib_path_arg = DeclareLaunchArgument(
        'right_calib_path',
        default_value='./model/right_calib.yaml',
        description='Path to the right calibration file'
    )
    inspire_port_arg = DeclareLaunchArgument(
        'inspire_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for the Inspire device'
    )

    # --- Node 1: glove_ros node ---
    # node1 = Node(
    #     package='glove_ros',
    #     executable='serial_node',
    #     output='screen',
    #     parameters=[
    #         {'left_port': LaunchConfiguration('left_port')},
    #         {'right_port': LaunchConfiguration('right_port')},
    #         {'inference_mode': True},
    #     ]
    # )

    # --- Node 2: glove_visualizer node ---
    node2 = Node(
        package='glove_visualizer',
        executable='hand_display_node',
        output='screen',
        arguments=[
            '--left-hand-calib', LaunchConfiguration('left_calib_path'),
            '--right-hand-calib', LaunchConfiguration('right_calib_path')
        ]
    )

    # --- Node 3: cyber_retargeting node ---
    node3 = Node(
        package='cyber_retarget_ros2_py',
        executable='retarget_node',
        output='screen',
        arguments=[
            '--finger-config', LaunchConfiguration('finger_config_path'),
            '--robot-config', LaunchConfiguration('robot_config_path')
        ]
    )

    # --- Node 4: inspire_node ---
    node4 = Node(
        package='inspire_retarget',
        executable='controller_node',
        output='screen',
        arguments=['-s', LaunchConfiguration('inspire_port')]
    )

    # --- Launch Description ---
    return LaunchDescription([
        # left_port_arg,
        # right_port_arg,
        finger_config_path_arg,
        robot_config_path_arg,
        left_calib_path_arg,
        right_calib_path_arg,
        inspire_port_arg,
        # node1,
        node2,
        node3,
        node4
    ])