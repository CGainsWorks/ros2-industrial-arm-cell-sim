from glob import glob
import os

from setuptools import find_packages, setup

package_name = "cell_grasping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Simulated depth camera, perception pipeline and the vision-guided grasp planner",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_planner_node = cell_grasping.nodes.grasp_planner_node:main",
        ],
    },
)
