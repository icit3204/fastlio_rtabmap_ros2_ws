from glob import glob
from setuptools import find_packages, setup


package_name = "wheelchair_cmd_adapter"

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
    description="Phase 4 mock-only TwistStamped to legacy wheelchair command adapter.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_wheelchair_cmd_adapter = wheelchair_cmd_adapter.mock_wheelchair_cmd_adapter:main",
            "phase4_p4d1_mock_preflight_runner = wheelchair_cmd_adapter.phase4_p4d1_mock_preflight_runner:main",
            "phase4_p4d2_matrix_runner = wheelchair_cmd_adapter.phase4_p4d2_matrix_runner:main",
        ],
    },
)
