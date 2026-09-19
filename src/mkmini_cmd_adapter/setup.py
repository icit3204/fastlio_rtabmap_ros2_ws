from setuptools import find_packages, setup


package_name = "mkmini_cmd_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/docs", [
            "docs/MKMINI_PROTOCOL_CONTRACT.md",
            "docs/MKMINI_RECEIVE_ONLY_CONTRACT.md",
            "docs/MKMINI_COMMAND_AUTHORITY_ZERO_TX.md",
        ]),
        ("share/" + package_name + "/config", [
            "config/mkmini_installed_unit_identity.yaml",
            "config/phase5_command_chain_dry_run.yaml",
            "config/r11_physical_backend_commissioning.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dog",
    maintainer_email="dog@example.invalid",
    description="Pure MK-mini codecs and ROS mock-only command adapter.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mkmini_cmd_adapter_node = mkmini_cmd_adapter.ros_node:main",
            "mkmini_command_chain_dry_run_node = mkmini_cmd_adapter.dry_run_node:main",
            "mkmini_controller_validity_node = mkmini_cmd_adapter.validity_ros_node:main",
            "mkmini_mk2e2a_zero_speed_runner = mkmini_cmd_adapter.zero_speed_runner:main",
            "mkmini_backend_receive_only = mkmini_cmd_adapter.backend_receive_only:main",
            "mkmini_backend_zero_heartbeat = mkmini_cmd_adapter.backend_zero_heartbeat:main",
            "mkmini_backend_first_ground_motion = mkmini_cmd_adapter.backend_first_ground_motion:main",
            "mkmini_alive_diagnostic = mkmini_cmd_adapter.alive_diagnostic:main",
            "mkmini_physical_ros_backend = mkmini_cmd_adapter.physical_ros_backend:main",
        ],
    },
)
