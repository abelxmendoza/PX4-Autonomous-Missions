from setuptools import find_packages, setup
import os
from glob import glob

package_name = "px4_offboard"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Abel",
    maintainer_email="abelxmendoza@gmail.com",
    description="PX4 OFFBOARD control nodes using px4_msgs and Micro XRCE-DDS",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "offboard_control = px4_offboard.offboard_control:main",
            "offboard_mission = px4_offboard.offboard_mission:main",
            "flight_trail = px4_offboard.flight_trail:main",
            "demo_hud = px4_offboard.demo_hud:main",
            "lidar_sectors = px4_offboard.lidar_sectors:main",
            "vv_replay = px4_offboard.vv_replay:main",
        ],
    },
)
