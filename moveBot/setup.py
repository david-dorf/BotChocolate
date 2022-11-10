from setuptools import setup

package_name = 'moveBot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oubre',
    maintainer_email='oubrejames@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['intercept = moveBot.intercept:main',
                            'moveBot = moveBot.moveBot:main',
                            'brick_intercept = moveBot.brick_intercept:main',
                            'intercept_msgs = moveBot.intercept_msgs:main'
        ],
    },
)
