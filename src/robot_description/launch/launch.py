import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #default urdf path
    urdf_package_path = get_package_share_directory('robot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot_test.urdf')
    #arg for urdf
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model',
        default_value=str(default_urdf_path),
        description='loading urdf path'
    )
    command_result = launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)
    action_robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2'
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])