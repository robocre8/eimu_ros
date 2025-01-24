import os

from ament_index_python.packages import get_package_share_directory # type: ignore

from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('eimu_ros'))
    eimu_ros_config_file = os.path.join(pkg_path,'config','eimu_ros_test_params.yaml')

    eimu_ros_node = Node(
        package='eimu_ros',
        executable='eimu_ros',
        name='eimu_ros',
        output='screen',
        parameters=[eimu_ros_config_file],
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(eimu_ros_node)
    
    return ld      # return (i.e send) the launch description for excecution
    
