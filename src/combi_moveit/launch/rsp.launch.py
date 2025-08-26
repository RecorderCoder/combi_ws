from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("combi", package_name="combi_moveit")
        # 필요 시 .robot_description(file_path="...") 등으로 xacro 경로 지정 가능
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
