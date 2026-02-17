from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'game_director'


def _discover_config_files() -> list[str]:
    package_dir = Path(__file__).resolve().parent
    candidates: list[Path] = [Path(path) for path in glob('config/*.yaml')]
    candidates.extend([
        Path('..') / '..' / 'config' / 'game_director.yaml',
        Path('..') / '..' / 'config' / 'task_pool.yaml',
    ])

    files: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        absolute = (package_dir / candidate).resolve()
        if not absolute.exists():
            continue
        resolved = str(absolute)
        if resolved in seen:
            continue
        seen.add(resolved)
        files.append(str(candidate))
    return files


config_files = _discover_config_files()
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
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
    description='Mission executive that dispatches and manages RMF game tasks.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_director=game_director.director_node:main',
        ],
    },
)
