from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions.find_package import FindPackageShare

def generate_launch_description():
    rviz_config=FindPackageShare.find(FindPackageShare,"pcd_reg")+"/launch/config/config.rviz"
    print(rviz_config)
    return LaunchDescription([
        Node(
            package='pcd_reg',
            executable='pcd_reg',
            name='pcd_reg',
            parameters=[{"input_pcd1_path":"data/capture0001.pcd"},
                        {"input_pcd2_path":"data/capture0002.pcd"},
                        {"out_pcd_path":"data/transformed.pcd"},
                        {"algorithm": "gicp"}]
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config])
    ])