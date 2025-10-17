# launch/mw_ahrs_launch.py (발췌)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) AHRS 드라이버
        Node(
            package="mw_ahrs_ros2",
            executable="mw_ahrs_driver_node",
            name="mw_ahrs_driver_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB0",
                "frame_id": "imu_link",
                "version": "v12",
                "use_realtime_pub": False
            }]
        ),

        # 2) 정적 TF: base_link -> imu_link
        #    인자: x y z roll pitch yaw parent child
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_imu_tf",
            arguments=["0.0", "0.0", "0.15",  # x y z (로봇에 장착한 위치)
                       "0.0", "0.0", "0.0",  # r p y (장착 각도 보정)
                       "base_link", "imu_link"]
        ),

        Node(
            package='mw_ahrs_ros2',
            executable='imu_orientation_tf_node',
            name='imu_orientation_tf',
            output='screen',
            parameters=[{
                'parent_frame': 'imu_link',
                'child_frame':  'imu_visual',
            }]
        ),
    ])
