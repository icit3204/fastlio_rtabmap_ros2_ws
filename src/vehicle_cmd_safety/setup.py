from glob import glob
from setuptools import find_packages, setup


package_name = "vehicle_cmd_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dog",
    maintainer_email="dog@example.invalid",
    description="Phase 4 generic vehicle command safety gate and collision validity monitor.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "guarded_vehicle_cmd_gate = vehicle_cmd_safety.guarded_vehicle_cmd_gate:main",
            "collision_monitor_validity_monitor = vehicle_cmd_safety.collision_monitor_validity_monitor:main",
            "phase4_p4c_permission_fixture = vehicle_cmd_safety.phase4_p4c_permission_fixture:main",
            "phase4_p4c_safe_twist_fixture = vehicle_cmd_safety.phase4_p4c_safe_twist_fixture:main",
            "phase4_p4c_evidence_monitor = vehicle_cmd_safety.phase4_p4c_evidence_monitor:main",
            "phase4_p4c_runtime_runner = vehicle_cmd_safety.phase4_p4c_runtime_runner:main",
        ],
    },
)
