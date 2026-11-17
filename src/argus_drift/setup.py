from glob import glob

from setuptools import setup

package_name = "argus_drift"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxjo",
    maintainer_email="maxjo@todo.todo",
    description="Optional vision-based drift correction between Argus modules",
    license="TODO",
    entry_points={
        "console_scripts": [
            "drift_node = argus_drift.drift_node:main",
        ],
    },
)
