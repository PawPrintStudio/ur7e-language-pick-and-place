from setuptools import find_packages, setup

package_name = "arm_language"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # The corpus ships inside the Python package rather than in share/, because
    # `python3 -m arm_language.eval` must work on a laptop with no ROS install
    # — there is no ament index to look the share path up in.
    package_data={package_name: ["corpus/*.json"]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikola Markovic",
    maintainer_email="nikolamarkovic.idea@gmail.com",
    description="Free-form text to a validated manipulation command (task 3.1).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "intent_parser_node = arm_language.intent_parser_node:main",
        ],
    },
)
