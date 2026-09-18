from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'logistics_amr_goal_dispatcher'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woozoo',
    maintainer_email='woozoo@todo.todo',
    description='Manual multi-robot Nav2 goal dispatcher for Logistics AMR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_robot_goal_dispatcher = '
            'logistics_amr_goal_dispatcher.goal_dispatcher:main',
        ],
    },
)
