from glob import glob
from setuptools import find_packages, setup


package_name = "inno_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/audio", glob("audio/*.wav")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeno04",
    maintainer_email="seeno04@example.com",
    description="Bringup for Camera Module 3, RPLIDAR C1, and RF2O odometry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "odom_to_path = inno_robot_bringup.odom_to_path_node:main",
        "tf_to_path = inno_robot_bringup.tf_to_path_node:main",
        "amcl_pose_tf_bridge = inno_robot_bringup.amcl_pose_tf_bridge:main",
        "auto_localization_supervisor = inno_robot_bringup.auto_localization_supervisor:main",
        "lifecycle_autostart = inno_robot_bringup.lifecycle_autostart:main",
        "slam_keyboard_runner = inno_robot_bringup.slam_keyboard_runner:main",
        "bag_topic_preflight = inno_robot_bringup.bag_topic_preflight:main",
        "mode3_audio_guide = inno_robot_bringup.mode3_audio_guide:main",
        "bno055_imu = inno_robot_bringup.bno055_imu_node:main",
    ]},
)
