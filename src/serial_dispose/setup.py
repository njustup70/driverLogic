from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'serial_dispose'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),

    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,

    data_files=[
        # ament 索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [
            'serial_dispose = serial_dispose.serial_dispose:main',
        ],
    },
)