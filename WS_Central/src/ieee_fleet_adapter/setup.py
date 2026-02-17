from glob import glob
from setuptools import find_packages, setup

package_name = 'ieee_fleet_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IEEE 2026',
    maintainer_email='maintainer@example.com',
    description='IEEE 2026 RMF fleet adapter (ground + drone).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=ieee_fleet_adapter.fleet_adapter:main',
        ],
    },
)
