from glob import glob

from setuptools import setup


package_name = "ekf_line_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Line-based EKF localization and waypoint control for AMR homework 4.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "ekf_line_localization = ekf_line_localization.ekf_line_localization_node:main",
            "ekf_waypoint_controller = ekf_line_localization.waypoint_controller:main",
        ],
    },
)
