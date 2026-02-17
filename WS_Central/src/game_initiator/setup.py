from glob import glob

from setuptools import find_packages, setup

package_name = 'game_initiator'

launch_files = glob('launch/*.py')
config_files = glob('config/*.yaml')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
    ('share/' + package_name + '/launch', launch_files),
]
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IEEE 2026',
    maintainer_email='maintainer@example.com',
    description='GPIO start input monitor for invoking game_director start service.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_initiator_node=game_initiator.game_initiator_node:main',
        ],
    },
)
