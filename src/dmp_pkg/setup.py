from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dmp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'pandas','scipy','matplotlib'],
    zip_safe=True,
    maintainer='tree',
    maintainer_email='swang2792@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "arm_trajectory_record = dmp_pkg.arm_trajectory_record:main",
            "trajectory_gen_pub = dmp_pkg.trajectory_gen_pub:main",
            "save_weight = dmp_pkg.save_weight:main",
        ],
    },
)
