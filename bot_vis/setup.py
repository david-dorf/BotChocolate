from setuptools import setup
from glob import glob
import os

package_name = 'bot_vis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oubre',
    maintainer_email='oubrejames@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['image_sub = bot_vis.image_sub:main',
        'april_rec = bot_vis.april_rec:main',
        'april_tf = bot_vis.april_tf:main',
        'calibration = bot_vis.calibration:main'
        ],
    },
)
