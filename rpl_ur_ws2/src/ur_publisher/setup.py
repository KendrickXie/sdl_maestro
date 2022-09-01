from setuptools import setup
from glob import glob

package_name = 'ur_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("launch/*.launch.py")),
        ('share/' + package_name + '/config', glob("config/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kendrick',
    maintainer_email='kendrick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "rpl_publisher_joint_trajectory_controller = \
                ur_publisher.rpl_publisher_joint_trajectory_controller:main",
            "rpl_ur_move = \
                ur_publisher.rpl_ur_move:main",
        ],
    },
)
