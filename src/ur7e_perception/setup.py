from glob import glob
from setuptools import setup

setup(
    name='ur7e_perception', version='0.1.0', packages=['ur7e_perception'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur7e_perception']),
        ('share/ur7e_perception', ['package.xml']),
        ('share/ur7e_perception/launch', glob('launch/*.launch.py')),
        ('share/ur7e_perception/config', glob('config/*.yaml') + glob('config/*.sdf')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Nikola Markovic', maintainer_email='nikolamarkovic.idea@gmail.com',
    description='On-demand registered RGB-D perception and camera-free replay', license='MIT',
    entry_points={'console_scripts': [
        'perception_node = ur7e_perception.nodes:perception_main',
        'rgbd_source = ur7e_perception.nodes:source_main',
        'perception_eval = ur7e_perception.evaluate:main',
    ]},
)
