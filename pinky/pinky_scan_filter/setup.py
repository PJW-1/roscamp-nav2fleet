from setuptools import setup

setup(
    name='pinky_scan_filter',
    version='0.1.0',
    packages=['pinky_scan_filter'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_scan_filter']),
        ('share/pinky_scan_filter', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'scan_filter = pinky_scan_filter.node:main',
    ]},
)
