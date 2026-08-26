import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # radar_bringup 包已拆除：config/ 直接位于工作空间顶层，按 launch 文件位置推导
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'mid70_default.yaml')
    # rviz2 配置文件（工作空间根目录），随 launch 一起打开，无需手动启动 rviz2
    rviz_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Mid-70_perception_module.rviz')

    enable_lidar = LaunchConfiguration('lidar', default='true')
    enable_align = LaunchConfiguration('align', default='false')
    enable_visualizer = LaunchConfiguration('visualizer', default='false')
    enable_rviz = LaunchConfiguration('rviz', default='true')

    tf_x = LaunchConfiguration('tf_x', default='0.0')
    tf_y = LaunchConfiguration('tf_y', default='0.0')
    tf_z = LaunchConfiguration('tf_z', default='0.0')
    tf_qx = LaunchConfiguration('tf_qx', default='0.0')
    tf_qy = LaunchConfiguration('tf_qy', default='0.0')
    tf_qz = LaunchConfiguration('tf_qz', default='0.0')
    tf_qw = LaunchConfiguration('tf_qw', default='1.0')

    # Livox Mid-70 驱动（根命名空间，节点名 livox_lidar → 话题 /pc_raw，帧 livox_lidar_frame）
    livox_node = Node(
        package='livox_lidar',
        executable='livox_lidar_node',
        name='livox_lidar',
        parameters=[config_path],
        condition=IfCondition(enable_lidar),
    )

    # world → livox_lidar_frame 静态 TF（雷达安装位姿）。
    # 注意：pc_detector 启动时阻塞等待该 TF，没有它节点会一直等待；
    # 启动 pc_aligner 时由对齐结果接管 world TF，不再发静态 TF。
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_livox_lidar',
        arguments=['--x', tf_x, '--y', tf_y, '--z', tf_z,
                   '--qx', tf_qx, '--qy', tf_qy, '--qz', tf_qz, '--qw', tf_qw,
                   '--frame-id', 'world', '--child-frame-id', 'livox_lidar_frame'],
        condition=UnlessCondition(enable_align),
    )

    # 点云检测 + 跟踪（根命名空间，与 livox 的相对话题 <lidar>/pc_raw 对应）
    detector_node = Node(
        package='pc_detector',
        executable='pc_detector_node',
        name='pc_detector',
        parameters=[config_path],
        output='both',
    )

    # 可选：点云对场地模型 ICP 对齐，发布 world TF
    aligner_node = Node(
        package='pc_aligner',
        executable='pc_aligner',
        name='pc_aligner',
        parameters=[config_path],
        output='both',
        condition=IfCondition(enable_align),
    )

    # 可选：跟踪目标可视化
    visualizer_node = Node(
        package='target_visualizer',
        executable='target_visualizer',
        name='target_visualizer',
        output='both',
        condition=IfCondition(enable_visualizer),
    )

    # rviz2 可视化（默认启动，加载 Mid-70_perception_module.rviz；无显示环境可 rviz:=false 关闭）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='启动 Livox Mid-70 驱动'),
        DeclareLaunchArgument('align', default_value='false', description='启动 pc_aligner（对齐后由它发布 world TF）'),
        DeclareLaunchArgument('visualizer', default_value='false', description='启动 target_visualizer'),
        DeclareLaunchArgument('rviz', default_value='true', description='启动 rviz2 可视化（加载 Mid-70_perception_module.rviz）'),
        DeclareLaunchArgument('tf_x', default_value='0.0', description='静态 TF world→livox_lidar_frame 平移 x (m)'),
        DeclareLaunchArgument('tf_y', default_value='0.0', description='静态 TF world→livox_lidar_frame 平移 y (m)'),
        DeclareLaunchArgument('tf_z', default_value='0.0', description='静态 TF world→livox_lidar_frame 平移 z (m)'),
        DeclareLaunchArgument('tf_qx', default_value='0.0', description='静态 TF world→livox_lidar_frame 旋转 qx'),
        DeclareLaunchArgument('tf_qy', default_value='0.0', description='静态 TF world→livox_lidar_frame 旋转 qy'),
        DeclareLaunchArgument('tf_qz', default_value='0.0', description='静态 TF world→livox_lidar_frame 旋转 qz'),
        DeclareLaunchArgument('tf_qw', default_value='1.0', description='静态 TF world→livox_lidar_frame 旋转 qw'),
        livox_node,
        static_tf_node,
        detector_node,
        aligner_node,
        visualizer_node,
        rviz_node,
    ])
