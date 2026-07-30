from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 获取包路径
    wheelchair_controller_share = FindPackageShare('wheelchair_controller')
    # 配置文件路径
    config_file = PathJoinSubstitution([
        wheelchair_controller_share,
        'config',
        'wheelchair_controller_param.yaml'
    ])
    # 底层轮椅控制节点
    wheelchair_controller_node = Node(
        package='wheelchair_controller',
        executable='wheelchair_controller_node',
        name='wheelchair_controller_node',
        output='screen',
        parameters=[
            config_file
        ],
        prefix='gnome-terminal --'  # 在新的终端窗口中运行该节点，方便查看输出日志
    )

    return LaunchDescription([
        wheelchair_controller_node
    ])