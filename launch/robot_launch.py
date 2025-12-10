import os

from launch import LaunchDescription, actions, launch_description_sources
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import EnvironmentVariable

### Hexman in .bashrc ###
# export HEXMAN_TYPE="echo"
# export HEXMAN_BASH_PATH=~/"sdk_"$HEXMAN_TYPE"_ws"/install/setup.bash
# source $HEXMAN_BASH_PATH
### Hexman - 1 ###

def generate_launch_description():

    robot_type = EnvironmentVariable('HEXMAN_TYPE', default_value='echo')
    urdf_pkg_path = FindPackageShare(["xpkg_urdf_", robot_type])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "model.urdf"]
    )
    description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]), 
        value_type=str
    )
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': description_content,
        }]
    )
    joint_state_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
                
    xnode_comm = Node(
        name="xnode_comm",
        package="xpkg_comm",
        executable="xnode_comm",
        output="screen",
        parameters=[{
            "dev_list": False,
            "com_enable": True,
            "com_channel_common": False,
            "com_channel_xstd": True,
        }]
    )

    vehicle_ini_path = PathJoinSubstitution(
        [FindPackageShare("xpkg_vehicle"), "ini", "device_id_list.ini"]
    )
    xnode_vehicle = Node(
        name="xnode_vehicle",
        package="xpkg_vehicle",
        executable="xnode_vehicle",
        output="screen",
        parameters=[{
            "ini_path": vehicle_ini_path,
            "show_path": True,
            "show_loc": False,
            "calc_speed": False,
            "mode_can_lock": False,
            "rate_x": 1.0,
            "rate_y": 1.0,
            "rate_z": 1.0,
            "rate_az": 1.0,
        }]
    )
    
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    scan_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type':channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen'
    )

    base_link_to_laser = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        # arguments = ["-0.043", "0", "0.2", "0", "0", "0", "base_link", "laser"]
        arguments = ["-0.043", "0", "0.2", "3.1415", "0", "0", "base_link", "laser"]
    )
         
    return LaunchDescription([
        robot_state_node, joint_state_node, xnode_comm, xnode_vehicle, scan_node, base_link_to_laser
    ])
