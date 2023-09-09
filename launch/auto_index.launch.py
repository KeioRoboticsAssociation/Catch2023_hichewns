import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir=get_package_share_directory("cal_rtheta")
    urdf=os.path.join(package_dir,"urdf","catch.urdf")
    csv=os.path.join(package_dir,"csv","pose.csv")

    return LaunchDescription([

        Node(
            package="cal_rtheta",
            executable='joint_publisher',
            name='joint_publisher',
            arguments=[urdf]),

        # Node(
        #     package="cal_rtheta",
        #     executable='state_index_forprevent',
        #     name='state_index_forprevent',
        #     arguments=[urdf]),

        # Node(
        #     package="cal_rtheta",
        #     executable='index',
        #     name='index',
        #     arguments=[urdf]),

        Node(
            package="cal_rtheta",
            executable='state_index',
            name='state_index',
            arguments=[urdf, csv]),

        Node(
            package="cal_rtheta",
            executable='xy_to_rtheta_index',
            name='xy_to_rtheta_index',
            arguments=[urdf]),
        
        # Node(
        #     package="cal_rtheta",
        #     executable='xy_to_rtheta_index_copy',
        #     name='xy_to_rtheta_index_copy',
        #     arguments=[urdf]),

        
        Node(
            package="cal_rtheta",
            executable='joysub_cal',
            name='joysub_cal',
            arguments=[urdf]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf],
            output="screen"),

        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     arguments=[urdf], 
        #     output="screen"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2"),
    ])