# combi_moveit/launch/bringup_with_rviz.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_move_group_launch,
)

def generate_launch_description():
    # MoveIt 설정 객체
    moveit_config = (
        MoveItConfigsBuilder("combi", package_name="combi_moveit")
        .to_moveit_configs()
    )

    # RViz 설정 파일 인자 (기본값: combi_moveit/config/moveit.rviz)
    pkg = FindPackageShare('combi_moveit')
    default_rviz = PathJoinSubstitution([pkg, 'config', 'moveit.rviz'])
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # 1) robot_state_publisher
    rsp_ld = generate_rsp_launch(moveit_config)

    # 2) move_group
    move_group_ld = generate_move_group_launch(moveit_config)

    # 3) rviz2 (원하는 rviz 설정 파일로 구동)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        # RViz가 로봇을 인식하려면 robot_description이 이미 파라미터 서버에 있어야 함
        # generate_rsp_launch 가 이를 올려줍니다.
    )

    ld = LaunchDescription()
    ld.add_action(rviz_config_arg)
    # 하위 LaunchDescription의 엔트리를 합쳐 넣기
    ld.extend(rsp_ld.entities)
    ld.extend(move_group_ld.entities)
    ld.add_action(rviz)
    return ld
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('nova2_delto_moveit')
    rviz_config = PathJoinSubstitution([pkg, 'config', 'moveit.rviz'])
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config],
             output='screen')
    ])

"""