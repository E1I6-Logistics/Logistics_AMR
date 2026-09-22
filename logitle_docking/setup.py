from setuptools import find_packages, setup

package_name = 'logitle_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhlee',
    maintainer_email='dl1wjd2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'precision_docking_ICP_server = logitle_docking.precision_docking_ICP_server:main',
            'precision_docking_ICP_align_server = logitle_docking.precision_docking_ICP_align_server:main',
            'precision_docking_ICP_align_latch_server = logitle_docking.precision_docking_ICP_align_latch_server:main',
            'precision_docking_ICP_align_server_V2 = logitle_docking.precision_docking_ICP_align_server_V2:main',

        ],
    },
)
