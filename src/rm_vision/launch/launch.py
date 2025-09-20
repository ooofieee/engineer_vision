import launch
import launch_ros
def generate_launch_description():
    action_node_redeem_box_node = launch_ros.actions.Node(
        package = 'engineer_vision_pkg',
        executable ='redeem_box_node',
        output = 'screen'
    )
    action_node_video_capturer_node = launch_ros.actions.Node(
        package='engineer_vision_pkg',
        executable='video_capturer_node',
        output='screen'
    )
    action_node_serial_node = launch_ros.actions.Node(
        package='engineer_vision_pkg',
        executable='serial_node',
        output='screen'
    )
    action_node_camera_node = launch_ros.actions.Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        output='screen'
    )
    return launch.LaunchDescription([
        action_node_redeem_box_node,
        action_node_video_capturer_node
       # action_node_serial_node,
        #action_node_camera_node
    ])