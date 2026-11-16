from glob import glob

from setuptools import setup

setup(
    name='pode_fsm',
    version='0.1.0',
    packages=['pode_fsm'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pode_fsm']),
        ('share/pode_fsm', ['package.xml']),
        ('share/pode_fsm/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'pode_fsm_node = pode_fsm.node:main',
    ]},
)
