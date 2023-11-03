import glob
import os
from setuptools import setup

package_name = "jetson_csi_camera_ros2_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "param"), glob.glob("param/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sydney Kim",
    maintainer_email="sydney@mechasolution.com",
    description="CSI Camera ROS2 Driver for Jetson",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "jetson_csi_camera_ros2_driver_node = jetson_csi_camera_ros2_driver.jetson_csi_camera_ros2_driver_node:main"
        ],
    },
)
