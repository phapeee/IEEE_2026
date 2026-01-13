from setuptools import setup

package_name = "limit_switch_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description=(
        "Configurable calibration node that sequences limit switch conditions, movements, and actions."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibration_node = limit_switch_calibration.calibration_node:main",
        ],
    },
)
