# combi_moveit/launch/demo.launch.py
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def _pkg_file(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)

def _load_yaml(pkg: str, rel: str):
    path = _pkg_file(pkg, rel)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg = FindPackageShare("combi_moveit")
    controllers_yaml = PathJoinSubstitution([pkg, "config", "ros2_controllers.yaml"])
    # ★ 네 파일명과 일치하도록 'combi.xacro'로 지정
    urdf_xacro = PathJoinSubstitution([pkg, "config", "combi.xacro"])
    rviz_cfg   = PathJoinSubstitution([pkg, "config", "moveit.rviz"])

    # 기존 코드
    # robot_description = {
    #    "robot_description": Command(["xacro ", urdf_xacro])
    # }

    # 수정된 코드 (예시)
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_xacro,
            # " parent_link:=", "Link6",      # 'parent_link' 인자를 "tool0"으로 변경
            " mount_xyz:=", "'0.001 0.001 .0016'",    # 'mount_xyz' 인자를 "0.1 0 0"으로 변경
            " mount_rpy:=", "'0.785398 1.570796 0'",   # 'mount_rpy' 인자를 "0 0 1.57"로 변경
            " gr_prefix:=", "dg3f_" # 'gr_prefix' 인자를 "new_gripper_"로 변경
        ])
    }

    # SRDF는 파일 '내용'을 그대로 전달
    with open(_pkg_file("combi_moveit", "config/combi.srdf"), "r") as f:
        srdf_text = f.read()
    robot_description_semantic = {
        "robot_description_semantic": srdf_text
    }

    # 기타 파라미터(YAML)
    kinematics_yaml   = _load_yaml("combi_moveit", "config/kinematics.yaml")
    joint_limits_yaml = _load_yaml("combi_moveit", "config/joint_limits.yaml")
    pilz_limits_yaml  = _load_yaml("combi_moveit", "config/pilz_cartesian_limits.yaml")
    moveit_ctrls_yaml = _load_yaml("combi_moveit", "config/moveit_controllers.yaml")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,           # ← combi.xacro에 <ros2_control>가 포함돼 있어야 함
            controllers_yaml
        ],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["nova2_group_controller"],
        output="screen",
    )

    spawn_grip = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["delto_controller"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            pilz_limits_yaml,
            moveit_ctrls_yaml,
            {"planning_pipelines": ["ompl"]}  # 필요시 pilz 추가
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        joint_limits_yaml,
        pilz_limits_yaml,
        moveit_ctrls_yaml,
        # ↓↓↓ 이 세 줄이 핵심: OMPL만 쓰도록 고정
        {"planning_plugin": "ompl_interface/OMPLPlanner",
         "default_planning_pipeline": "ompl",
         "planning_pipelines": ["ompl"]},
        # (있다면) OMPL 세부 설정 파일도 같이 로드
        # _load_yaml("combi_moveit", "config/ompl_planning.yaml"),
        ],
    )

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_cfg],
    #     output="screen",
    #     parameters=[robot_description, robot_description_semantic],
    # )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,      # ★ 추가
            joint_limits_yaml,    # (선택) 추가
            # pilz_limits_yaml,   # (선택) 추가
        ],
    )

    # return LaunchDescription([rsp, move_group, rviz])
    return LaunchDescription([
    rsp,
    control_node,         # ★ controller_manager 먼저
    spawn_jsb,            # ★ JSB → 팔 → 그리퍼 순서
    spawn_arm,
    spawn_grip,
    move_group,
    rviz,
])
