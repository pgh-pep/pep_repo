import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pep_pkg"

# Add new directories as needed
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append((os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")))
data_files.append(
    (
        os.path.join("share", package_name, "models"),
        glob("models/*.urdf.xacro") + glob("models/*.xacro"),
    )
)

# Add new nodes to entrypoints
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Varun Patel",
    maintainer_email="varunpat789@gmail.com",
    description="Core PEP autonomous package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
