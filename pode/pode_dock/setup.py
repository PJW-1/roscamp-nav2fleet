from setuptools import setup

setup(
    name='pode_dock',
    version='0.1.0',
    packages=['pode_dock'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pode_dock']),
        ('share/pode_dock', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
