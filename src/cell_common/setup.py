import os

from setuptools import find_packages, setup

package_name = "cell_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Shared frames, transforms and the typed reader for the generated cell description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        ],
    },
)
