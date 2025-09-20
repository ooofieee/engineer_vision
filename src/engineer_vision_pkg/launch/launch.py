import launch
import launch_ros
def generate_launch_description():
    action_node_redeem_box_node = launch_ros.actions.Node(
        package = 'engineer_vision_pkg',
        executable ='RB_detector_node',
        output = 'screen'
    )
    action_node_video_capturer_node = launch_ros.actions.Node(
        package='engineer_vision_pkg',
        executable='video_capturer_node',
        output='screen'
    )
    action_node_video_saver_node = launch_ros.actions.Node(
        package='engineer_vision_pkg',
        executable='video_saver_node',
        output='screen'
    )
    action_node_RB_builder_node = launch_ros.actions.Node(
        package='arm_controller_pkg',
        executable='RB_builder_node',
        output='screen'
    )
    return launch.LaunchDescription([
        action_node_redeem_box_node,
        action_node_video_capturer_node,
        action_node_RB_builder_node
        #action_node_video_saver_node

    ])