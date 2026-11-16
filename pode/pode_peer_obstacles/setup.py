from setuptools import setup

setup(
    name='pode_peer_obstacles',
    version='0.1.0',
    packages=['pode_peer_obstacles'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pode_peer_obstacles']),
        ('share/pode_peer_obstacles', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'peer_obstacles = pode_peer_obstacles.node:main',
    ]},
)
