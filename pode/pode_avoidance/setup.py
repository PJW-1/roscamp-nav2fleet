from setuptools import setup

setup(
    name='pode_avoidance',
    version='0.1.0',
    packages=['pode_avoidance'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pode_avoidance']),
        ('share/pode_avoidance', ['package.xml']),
        ('share/pode_avoidance/params', ['params/avoidance.yaml']),
        ('share/pode_avoidance/launch', ['launch/avoidance.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'vision_stand_in = pode_avoidance.vision_stand_in:main',
    ]},
)
