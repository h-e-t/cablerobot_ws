from setuptools import find_packages, setup

package_name = "cablerobot_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="h-e-t",
    maintainer_email="h-e-t@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cablerobot_control_node = cablerobot_pkg.cablerobot_control_node:main",
            "teleop_node = cablerobot_pkg.teleop_node:main",
            "path_publisher_node = cablerobot_pkg.path_publisher_node:main",
        ],
    },
)
