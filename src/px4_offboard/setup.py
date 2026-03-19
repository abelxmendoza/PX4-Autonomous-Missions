from setuptools import find_packages, setup

package_name = "px4_offboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Abel",
    maintainer_email="abelxmendoza@gmail.com",
    description="PX4 OFFBOARD control nodes using px4_msgs and Micro XRCE-DDS",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "offboard_control = px4_offboard.offboard_control:main",
            "offboard_mission = px4_offboard.offboard_mission:main",
        ],
    },
)
