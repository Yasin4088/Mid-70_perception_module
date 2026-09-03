import os
import yaml

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def load_config(config_path):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def load_node_params(config, node_name):
    return config.get(node_name, {}).get('ros__parameters', {})


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def load_tf_mode(launch_params):
    tf_mode = str(launch_params.get('tf_mode', '')).strip().lower()
    if tf_mode in ('static', 'align'):
        return tf_mode
    if tf_mode:
        print(f"[mid70_launch] Unknown tf_mode '{tf_mode}', fallback to 'static'.")
    return 'static'


def load_static_tf_params(config_path):
    defaults = {
        'tf_x': 0.0,
        'tf_y': 0.0,
        'tf_z': 0.0,
        'tf_qx': 0.0,
        'tf_qy': 0.0,
        'tf_qz': 0.0,
        'tf_qw': 1.0,
    }
    config = load_config(config_path)
    params = load_node_params(config, '/static_tf_world_livox_lidar')
    return {key: str(params.get(key, value)) for key, value in defaults.items()}


def generate_launch_description():
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'mid70_default.yaml')
    rviz_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Mid-70_perception_module.rviz')
    config = load_config(config_path)
    launch_params = load_node_params(config, '/mid70_launch')
    static_tf = load_static_tf_params(config_path)

    enable_lidar = to_bool(launch_params.get('enable_lidar', True))
    enable_detector = to_bool(launch_params.get('enable_detector', True))
    tf_mode = load_tf_mode(launch_params)
    enable_static_tf = tf_mode == 'static'
    enable_align = tf_mode == 'align'
    enable_visualizer = to_bool(launch_params.get('enable_visualizer', False))
    enable_rviz = to_bool(launch_params.get('enable_rviz', False))

    # Livox Mid-70 驱动
    livox_node = Node(
        package='livox_lidar',
        executable='livox_lidar_node',
        name='livox_lidar',
        parameters=[config_path],
    )

    # world → livox_lidar_frame 静态 TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_livox_lidar',
        arguments=['--x', static_tf['tf_x'], '--y', static_tf['tf_y'], '--z', static_tf['tf_z'],
                   '--qx', static_tf['tf_qx'], '--qy', static_tf['tf_qy'], '--qz', static_tf['tf_qz'], '--qw', static_tf['tf_qw'],
                   '--frame-id', 'world', '--child-frame-id', 'livox_lidar_frame'],
    )

    # 点云检测 + 跟踪
    detector_node = Node(
        package='pc_detector',
        executable='pc_detector_node',
        name='pc_detector',
        parameters=[config_path],
        output='both',
    )

    # 点云对场地模型 ICP 对齐，发布 world TF
    aligner_node = Node(
        package='pc_aligner',
        executable='pc_aligner',
        name='pc_aligner',
        parameters=[config_path],
        output='both',
    )

    # 跟踪目标可视化
    visualizer_node = Node(
        package='target_visualizer',
        executable='target_visualizer',
        name='target_visualizer',
        parameters=[config_path],
        output='both',
    )

    # rviz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    # 启动节点
    launch_entities = []
    if enable_lidar:
        launch_entities.append(livox_node)
    if enable_static_tf:
        launch_entities.append(static_tf_node)
    if enable_detector:
        if enable_align:
            launch_entities.append(TimerAction(period=3.0, actions=[detector_node]))
        else:
            launch_entities.append(detector_node)
    if enable_align:
        launch_entities.append(aligner_node)
    if enable_visualizer:
        launch_entities.append(visualizer_node)
    if enable_rviz:
        if enable_align:
            launch_entities.append(TimerAction(period=3.0, actions=[rviz_node]))
        else:
            launch_entities.append(rviz_node)

    return LaunchDescription(launch_entities)
