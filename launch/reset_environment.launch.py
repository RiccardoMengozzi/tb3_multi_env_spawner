import os
import yaml
import random
from pathlib import Path

from tb3_multi_env_spawner import utils

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def load_yaml(file_path):
    """
    Load a YAML file and return its content as a dictionary.

    :param file_path: Path to the YAML file.
    :return: Parsed YAML content as a dictionary.
    """
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    """
    Generate the launch description for spawning multiple environments with robots in Gazebo.

    The function reads configurations from a YAML file, validates the input, and dynamically creates
    environments, spawns robots, and sets up other necessary nodes such as Cartographer and RViz.

    :return: LaunchDescription object containing all launch actions.
    """

    # Package and workspace details
    package_name = "tb3_multi_env_spawner"
    workspace_dir = Path(get_package_share_directory(package_name)).resolve().parents[3]

    # Paths to directories and files
    world_path = os.path.join(workspace_dir, "src", package_name, "worlds")
    models_properties_dir = os.path.join(
        workspace_dir, "src", package_name, "extras", "models_properties"
    )

    # Paths to directories and files
    models_properties_dir = os.path.join(
        workspace_dir, "src", package_name, "extras", "models_properties"
    )
    rviz_config_dir = os.path.join(workspace_dir, "src", package_name, "rviz")

    yaml_file_path = os.path.join(
        workspace_dir, "src", package_name, "config", "launch_params.yaml"
    )

    # Load launch parameters from YAML
    params = load_yaml(yaml_file_path)

    # Extract parameters from the YAML file
    gui = params["gazebo"]["gui"]
    verbose = params["launch"]["verbose"]
    gz_verbose = params["gazebo"]["verbose"]
    num_envs = params["env"]["num_envs"]
    mode = params["env"]["mode"]
    env_available_models = params["env"]["available_models"]
    env_model = params["env"]["model"]
    env_models = params["env"]["models"]
    random_pose = params["robot"]["random_pose"]
    robot_pos_x = params["robot"]["pose"]["x"]
    robot_pos_y = params["robot"]["pose"]["y"]
    robot_pos_yaw = params["robot"]["pose"]["yaw"]
    map_resolution = params["cartographer"]["map_resolution"]
    map_publish_period = params["cartographer"]["map_publish_period"]

    # Log launch parameters if verbose mode is enabled
    if verbose:
        print("\n\n----------------- LAUNCH PARAMETERS -----------------\n")
        print(f"[INFO] [{package_name}.launch.py] GUI: {gui}")
        print(f"[INFO] [{package_name}.launch.py] Number of environments: {num_envs}")
        print(f"[INFO] [{package_name}.launch.py] Mode: {mode}")
        print(f"[INFO] [{package_name}.launch.py] Available models: {env_available_models}")
        print(f"[INFO] [{package_name}.launch.py] Model: {env_model}")
        print(f"[INFO] [{package_name}.launch.py] Models: {env_models}")
        print(f"[INFO] [{package_name}.launch.py] Random pose: {random_pose}")
        print(f"[INFO] [{package_name}.launch.py] Robot pose x: {robot_pos_x}")
        print(f"[INFO] [{package_name}.launch.py] Robot pose y: {robot_pos_y}")
        print(f"[INFO] [{package_name}.launch.py] Robot pose yaw: {robot_pos_yaw}")
        print("\n----------------------------------------------------\n")

    # Input validation
    if num_envs < 1:
        raise ValueError("Number of environments should be greater than 0.")
    if mode not in ["single_model", "random_models", "multiple_models"]:
        raise ValueError(
            "Invalid mode. Please choose one of the following: 'single_model', 'random_models', 'multiple_models'."
        )
    if len(env_available_models) == 0:
        raise ValueError(
            "No models available for spawning. Please add models to the 'available_models' list in the launch_params.yaml file."
        )
    if len(env_models) != num_envs and mode == "multiple_models":
        raise ValueError(
            "Number of models in the 'models' list should be equal to the number of environments. \n"
            "Modify the 'models' list in the launch_params.yaml file."
        )
    if any([model not in env_available_models for model in env_models]):
        raise ValueError(
            "One or more models in the 'models' list are not available in the 'available_models' list."
        )

    # Prepare list of models based on the selected mode
    if mode == "random_models":
        env_models = [random.choice(env_available_models) for _ in range(num_envs)]
        if verbose:
            print(
                f"[INFO] [{package_name}.launch.py] Randomly selected models: {env_models}"
            )

    if mode == "single_model":
        env_models = [env_model for _ in range(num_envs)]

    # Create the world file dynamically
    world_name = utils.create_multi_env_world(
        num_envs, env_models, package_name, mode=mode
    )
    world_path = os.path.join(world_path, world_name)

    # Load robot URDF and SDF files
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

    robot_urdf_folder = f"turtlebot3_{TURTLEBOT3_MODEL}"
    robot_urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        robot_urdf_folder,
        "model.sdf",
    )

    # Launch actions list
    launch_actions = []


    existing_gazebo_models_paths = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_gazebo_models_paths = (
        f"{workspace_dir}/src/{package_name}/models/aws_models:"
        f"{workspace_dir}/src/{package_name}/models/fuel_models:"
        f"{workspace_dir}/src/{package_name}/models/turtlebot3_bighouse_model"
    )

    full_gazebo_models_paths = existing_gazebo_models_paths + new_gazebo_models_paths

    # Set Gazebo model paths
    set_gazebo_models_path_cmd = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=full_gazebo_models_paths
    )

    launch_actions.extend([set_gazebo_models_path_cmd])

    # Generate environment centers
    envs_centers = utils.generate_centers(
        num_envs, env_models, models_properties_dir=models_properties_dir
    )

    reset_gazebo_cmd = Node(
        package=package_name,
        executable="reset_gazebo",
        output="screen",
        parameters=[
            {
                "num_envs": num_envs,
                "robot_name": "tb3",
                "world_path": world_path,
                "gui": gui,
                "verbose": gz_verbose,
                "cartographer_config_path": os.path.join(
                    get_package_share_directory("turtlebot3_cartographer"), "config"
                ),
                "cartographer_config_basename": "turtlebot3_lds_2d.lua",
                "map_resolution": map_resolution,
                "map_publish_period": map_publish_period
            }
        ],
    )

    launch_actions.extend([reset_gazebo_cmd])

    # Create and configure nodes for each environment
    for i in range(num_envs):
        namespace = f"env_{i}"
        env_center = envs_centers[i]
        env_model = env_models[i]
        env_model_properties_path = os.path.join(
            models_properties_dir, f"{env_model}_properties.json"
        )
        rviz_config_file = utils.create_rviz_config(rviz_config_dir, namespace)

        # Reset environment service node
        reset_env_cmd = Node(
            package=package_name,
            executable="reset_environment",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "robot_name": "tb3",
                    "robot_namespace": namespace,
                    "robot_urdf_path": robot_urdf_path,
                    "env_center": env_center,
                    "env_model_properties_path": env_model_properties_path,
                    "cartographer_config_path": os.path.join(
                        get_package_share_directory("turtlebot3_cartographer"), "config"
                    ),
                    "cartographer_config_basename": "turtlebot3_lds_2d.lua",
                    "rviz_config_path": rviz_config_file,
                    "map_resolution": map_resolution,
                    "map_publish_period": map_publish_period
                }
            ],
        )

        launch_actions.extend([reset_env_cmd])

    return LaunchDescription(launch_actions)
