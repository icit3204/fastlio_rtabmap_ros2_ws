from glob import glob
from setuptools import find_packages, setup

package_name = "parking_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dog",
    maintainer_email="dog@example.invalid",
    description="Isolated Phase 2 Core Simple Nav2 baseline with fake base.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "phase2_fake_base = parking_robot_bringup.phase2_fake_base:main",
            "phase2_goal_test_runner = parking_robot_bringup.phase2_goal_test_runner:main",
        ],
    },
)
