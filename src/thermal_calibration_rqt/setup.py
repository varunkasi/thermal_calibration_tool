from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'thermal_calibration_rqt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/thermal_calibration.perspective']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RQT plugin for thermal camera calibration',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'thermal_calibration_rqt = thermal_calibration_rqt.thermal_calibration_plugin:main',
            'thermal_calibration_node = thermal_calibration_rqt.thermal_calibration_node:main',
        ],
        'rqt_gui.plugins': [
            'thermal_calibration = thermal_calibration_rqt.thermal_calibration_plugin:ThermalCalibrationPlugin',
        ],
    },
)