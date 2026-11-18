# ---- pode_plan_keep 패키지 설치 설정 ----
from setuptools import setup

setup(
    name='pode_plan_keep',
    version='0.1.0',
    packages=['pode_plan_keep'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pode_plan_keep']),
        ('share/pode_plan_keep', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'plan_keep = pode_plan_keep.node:main',
    ]},
)
