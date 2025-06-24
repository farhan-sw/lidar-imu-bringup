import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Arguments for hardware configuration
    use_lidar_arg = DeclareLaunchArgument(
        name='use_lidar',
        default_value='true',
        description='Whether to use the LIDAR'
    )
    use_lidar = LaunchConfiguration("use_lidar")
    
    use_imu_arg = DeclareLaunchArgument(
        name='use_imu',
        default_value='true',
        description='Whether to use the IMU'
    )
    use_imu = LaunchConfiguration("use_imu")
    
    # LIDAR configuration arguments
    lidar_serial_port_arg = DeclareLaunchArgument(
        name='lidar_serial_port',
        default_value='/dev/rplidar',
        description='Serial port for the LIDAR'
    )
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    
    lidar_baudrate_arg = DeclareLaunchArgument(
        name='lidar_baudrate',
        default_value='460800',
        description='Baudrate for the LIDAR serial communication'
    )
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    
    lidar_frame_id_arg = DeclareLaunchArgument(
        name='lidar_frame_id',
        default_value='laser',
        description='Frame ID for the LIDAR'
    )
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    
    lidar_inverted_arg = DeclareLaunchArgument(
        name='lidar_inverted',
        default_value='false',
        description='Whether to invert LIDAR scan data'
    )
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    
    lidar_angle_compensate_arg = DeclareLaunchArgument(
        name='lidar_angle_compensate',
        default_value='true',
        description='Whether to enable angle compensation for LIDAR'
    )
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    
    lidar_scan_mode_arg = DeclareLaunchArgument(
        name='lidar_scan_mode',
        default_value='Standard',
        description='Scan mode for the LIDAR'
    )
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")
    
    # IMU configuration arguments
    imu_port_arg = DeclareLaunchArgument(
        name='imu_port',
        default_value='/dev/imu_usb',
        description='Port for the IMU'
    )
    imu_port = LaunchConfiguration("imu_port")
    
    imu_baudrate_arg = DeclareLaunchArgument(
        name='imu_baudrate',
        default_value='9600',
        description='Baudrate for the IMU communication'
    )
    imu_baudrate = LaunchConfiguration("imu_baudrate")

    # LAUNCH HARDWARE    
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sllidar_ros2"), 
                "launch", 
                "sllidar_c1_launch.py"
            )
        ),
        launch_arguments={
            'serial_port': lidar_serial_port,
            'serial_baudrate': lidar_baudrate,
            'frame_id': lidar_frame_id,
            'inverted': lidar_inverted,
            'angle_compensate': lidar_angle_compensate,
            'scan_mode': lidar_scan_mode,
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )
    
    imu = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[
            {'port': imu_port},
            {'baud': imu_baudrate}
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_imu")),
    )
        
    return LaunchDescription([
        use_lidar_arg,
        use_imu_arg,
        lidar_serial_port_arg,
        lidar_baudrate_arg,
        lidar_frame_id_arg,
        lidar_inverted_arg,
        lidar_angle_compensate_arg,
        lidar_scan_mode_arg,
        imu_port_arg,
        imu_baudrate_arg,
        lidar,
        imu
    ])