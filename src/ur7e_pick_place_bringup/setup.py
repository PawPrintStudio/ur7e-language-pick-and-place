from glob import glob

from setuptools import find_packages, setup

package_name = "ur7e_pick_place_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/srdf", glob("srdf/*.xacro")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikola Markovic",
    maintainer_email="nikolamarkovic.idea@gmail.com",
    description="Phase 1 bringup: our deltas on the vendored UR7e+RG2 stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planning_scene = ur7e_pick_place_bringup.planning_scene:main",
        ],
    },
)
