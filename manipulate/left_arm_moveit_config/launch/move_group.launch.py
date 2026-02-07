from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("walkie_bot", package_name="left_arm_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )
    
    return generate_move_group_launch(moveit_config)