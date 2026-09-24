from setuptools import find_packages, setup

package_name = "ur7e_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikola Markovic",
    maintainer_email="nikolamarkovic.idea@gmail.com",
    description="motion_node: ExecutePrimitive action server over pymoveit2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_node = ur7e_motion.motion_node:main",
        ],
    },
)
