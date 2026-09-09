from setuptools import setup

package_name = 'operator_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/operator_gui.launch.py',
            'launch/operator_gui_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'operator_gui = operator_gui.main:main',
            'operator_gui_mock_fixture = operator_gui.mock_fixture:main',
        ],
    },
)
