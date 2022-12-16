import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = os.environ.get("ROS_NAMESPACE", "")

# TODO : param 사용하는 launch랑 사용 안하는 launch 분리할지 말지 고민중...
# (https://github.com/klintan/ros2_usb_camera/blob/foxy-devel/launch/usb_camera_component.launch.py)


def generate_launch_description():
    # 카메라 publishing 노드
    jetson_csi_camera_ros2_driver_params = LaunchConfiguration(
        "jetson_csi_camera_ros2_driver_params",
        default=os.path.join(
            get_package_share_directory("jetson_csi_camera_ros2_driver"),
            "param",
            "jetson_csi_camera_ros2_driver.yaml",
        ),
    )
    jetson_csi_camera_ros2_driver_arg = DeclareLaunchArgument(
        "jetson_csi_camera_ros2_driver_params",
        default_value=jetson_csi_camera_ros2_driver_params,
    )
    jetson_csi_camera_ros2_driver = Node(
        namespace=NAMESPACE,
        package="jetson_csi_camera_ros2_driver",
        executable="jetson_csi_camera_ros2_driver_node",
        name="jetson_csi_camera_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[jetson_csi_camera_ros2_driver_params],
    )

    return LaunchDescription(
        [jetson_csi_camera_ros2_driver_arg, jetson_csi_camera_ros2_driver]
    )
