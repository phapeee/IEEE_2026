from setuptools import setup

package_name = "holonomic_pi_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Simple holonomic PI controller that consumes NavigateToPose goals and publishes cmd_vel.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "holonomic_pi_controller_node = holonomic_pi_controller.holonomic_pi_controller_node:main",
        ],
    },
)
