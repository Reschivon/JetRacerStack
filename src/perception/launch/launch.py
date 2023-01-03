from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""
    nodes = [
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', get_package_share_directory('perception') + '/config/zed_rviz.rviz'],
            output='both',
        ),

        # # ZED Wrapper node
        # IncludeLaunchDescription(
        #     launch_description_source=PythonLaunchDescriptionSource([
        #         get_package_share_directory('zed_wrapper'),
        #         '/launch/include/zed_camera.launch.py'
        #     ]),
        #     launch_arguments={
        #         'camera_model': 'zed2',
        #         'config_path': [get_package_share_directory('perception'), 'config/custom.yaml'],
        #         'publish_urdf': false,    
        # }.items()
        # )
    ]

    return LaunchDescription(
        nodes
    )
    
    # container = LoadComposableNodes(
    #     target_container='perception_pipeline',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #              package='image_tools',
    #             plugin='image_tools::Cam2Image',
    #             name='cam2image',
    #             remappings=[('/image', '/burgerimage')],
    #             parameters=[{'width': 320, 'height': 240, 'burger_mode': True, 'history': 'keep_last'}],
    #             extra_arguments=[{'use_intra_process_comms': True}],
    #         ),
    #         ComposableNode(
    #             package='image_tools',
    #             plugin='image_tools::ShowImage',
    #             name='showimage',
    #             remappings=[('/image', '/burgerimage')],
    #             parameters=[{'history': 'keep_last'}],
    #             extra_arguments=[{'use_intra_process_comms': True}]
    #         ),
    #     ],
    # )

    return launch.LaunchDescription([container])