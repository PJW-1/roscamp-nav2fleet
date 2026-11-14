from setuptools import setup

setup(
    name='pinky_avoidance',
    version='0.1.0',
    packages=['pinky_avoidance'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_avoidance']),
        ('share/pinky_avoidance', ['package.xml']),
        ('share/pinky_avoidance/params', ['params/avoidance.yaml']),
        ('share/pinky_avoidance/launch', ['launch/avoidance.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'vision_stand_in = pinky_avoidance.vision_stand_in:main',
    ]},
)
