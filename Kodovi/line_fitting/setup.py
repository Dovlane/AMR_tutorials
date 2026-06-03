from glob import glob

from setuptools import setup

package_name = "line_fitting"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="ROS 2 Humble laser-scan line fitting example.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "line_fitting_node = line_fitting.line_fitting_node:main",
        ],
    },
)
