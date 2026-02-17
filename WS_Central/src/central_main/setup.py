import os
from pathlib import Path

from setuptools import setup

package_name = "central_main"

PACKAGE_DIR = Path(__file__).resolve().parent
BASE_DIR = PACKAGE_DIR.parents[1]
CONFIG_DIR = BASE_DIR / "config"

config_files = []
if CONFIG_DIR.is_dir():
    config_files = sorted(
        os.path.relpath(path, PACKAGE_DIR) for path in CONFIG_DIR.glob("*.yaml")
    )

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (
        "share/" + package_name + "/launch",
        [
            "launch/central_main.launch.py",
        ],
    ),
]
if config_files:
    data_files.append(("share/" + package_name + "/config", config_files))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Central hub launcher for RMF core stack and fleet adapters.",
    license="Apache-2.0",
    tests_require=[],
)
