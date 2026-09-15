from glob import glob

from setuptools import find_packages, setup

package_name = "ur7e_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Marker file so ROS 2 tooling (ros2 launch, ros2 pkg) can find the
        # package in the ament index.
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files and the factory kinematics calibration are *installed*
        # so FindPackageShare resolves them at runtime on any machine —
        # a raw repo path would only work where the repo layout is identical.
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikola Markovic",
    maintainer_email="nikolamarkovic.idea@gmail.com",
    description="Single-command UR7e bringup: driver + calibration + our defaults.",
    license="MIT",
    tests_require=["pytest"],
)
