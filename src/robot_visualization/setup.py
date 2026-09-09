from setuptools import setup

package_name = 'robot_visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/canonical_navigation.rviz']),
        ('share/' + package_name + '/launch', [
            'launch/navigation_visualization.launch.py',
            'launch/operator_visualization_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'navigation_visualization_helper = robot_visualization.navigation_visualization_helper:main',
            'visualization_fixture = robot_visualization.visualization_fixture:main',
            'integrated_demo_fixture = robot_visualization.integrated_demo_fixture:main',
        ],
    },
)
