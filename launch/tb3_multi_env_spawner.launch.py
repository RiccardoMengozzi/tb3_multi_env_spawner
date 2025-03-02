import os
import yaml
import random
import numpy as np
from pathlib import Path

import utils.utils as utils

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def load_yaml(file_path):
    """
    Load a YAML file and return its content as a dictionary.

    :param file_path: Path to the YAML file.
    :return: Parsed YAML content as a dictionary.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    """
    Generate the launch description for spawning multiple environments with robots in Gazebo.

    The function reads configurations from a YAML file, validates the input, and dynamically creates
    environments, spawns robots, and sets up other necessary nodes such as Cartographer and RViz.

    :return: LaunchDescription object containing all launch actions.
    """

    # Package and workspace details
    package_name = 'tb3_multi_env_spawner'
    workspace_dir = Path(get_package_share_directory(package_name)).resolve().parents[3]

    # Paths to directories and files
    world_path = os.path.join(workspace_dir, 'src', package_name, 'worlds')
    models_properties_dir = os.path.join(workspace_dir, 'src', package_name, 'extras', 'models_properties')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    rviz_config_dir = os.path.join(workspace_dir, 'src', package_name, 'rviz')

    yaml_file_path = os.path.join(
        workspace_dir,
        'src',
        package_name,
        'config',
        'launch_params.yaml'
    )

    # Load launch parameters from YAML
    params = load_yaml(yaml_file_path)

    # Extract parameters from the YAML file
    gui = params['gazebo']['gui']
    verbose = params['launch']['verbose']
    use_cartographer = params['cartographer']['enable']
    map_resolution = params['cartographer']['map_resolution']
    map_publish_period = params['cartographer']['map_publish_period']
    gz_verbose = params['gazebo']['verbose']
    num_envs = params['env']['num_envs']
    mode = params['env']['mode'] 
    env_available_models = params['env']['available_models']
    env_model = params['env']['model']
    env_models = params['env']['models']
    random_pose = params['robot']['random_pose']
    robot_pos_x = params['robot']['pose']['x']
    robot_pos_y = params['robot']['pose']['y']
    robot_pos_yaw = params['robot']['pose']['yaw']

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
    if mode not in ['single_model', 'random_models', 'multiple_models']:
        raise ValueError("Invalid mode. Please choose one of the following: 'single_model', 'random_models', 'multiple_models'.")
    if len(env_available_models) == 0:
        raise ValueError("No models available for spawning. Please add models to the 'available_models' list in the launch_params.yaml file.")
    if len(env_models) != num_envs and mode == 'multiple_models':
        raise ValueError("Number of models in the 'models' list should be equal to the number of environments. \n                         Modify the 'models' list in the launch_params.yaml file.")
    if any([model not in env_available_models for model in env_models]):
        raise ValueError("One or more models in the 'models' list are not available in the 'available_models' list.")

    # Prepare list of models based on the selected mode
    if mode == 'random_models':
        env_models = [random.choice(env_available_models) for _ in range(num_envs)]
        if verbose:
            print(f"[INFO] [{package_name}.launch.py] Randomly selected models: {env_models}")

    if mode == 'single_model':
        env_models = [env_model for _ in range(num_envs)]

    # Create the world file dynamically
    world_name = utils.create_multi_env_world(num_envs, 
                                              env_models,
                                              package_name,
                                              mode=mode)
    world = os.path.join(world_path, world_name)

    # Load robot URDF and SDF files
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    urdf_file_name = f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_urdf_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    robot_urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        robot_urdf_folder,
        'model.sdf'
    )

    # Launch actions list
    launch_actions = []

    existing_gazebo_models_paths = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_gazebo_models_paths = (
        f"{workspace_dir}/src/{package_name}/models/aws_models:"
        f"{workspace_dir}/src/{package_name}/models/fuel_models:"
        f"{workspace_dir}/src/{package_name}/models/turtlebot3_bighouse_model"
    )

    full_gazebo_models_paths = existing_gazebo_models_paths + new_gazebo_models_paths

    # Set Gazebo model paths
    set_gazebo_models_path_cmd = SetEnvironmentVariable(
        name ='GAZEBO_MODEL_PATH',
        value = full_gazebo_models_paths
    )

    # Declare launch argument for Cartographer
    declare_use_cartographer_cmd = DeclareLaunchArgument(
        'use_cartographer',
        default_value=str(use_cartographer),
        description='Whether to use cartographer or not'
    )

    # Include Gazebo server and client launch files
    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': str(gz_verbose)
        }.items()
    )

    gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(str(gui)),
        launch_arguments={
            'verbose': str(gz_verbose)
        }.items()
    )

    # Reset environment service node
    reset_env_cmd = Node(
        package=package_name,
        executable='reset_environment',
        output='screen',
    )


    # Generate environment centers
    envs_centers = utils.generate_centers(num_envs, env_models, models_properties_dir=models_properties_dir)
    print(f"[INFO] [{package_name}.launch.py] Environment centers: {envs_centers}")

    # Publisher for the environments properties
    envs_properties_publisher_cmd = Node(
        package=package_name,
        executable='envs_properties_publisher',
        output='screen',
        parameters=[{
            'envs_properties_dir_path': models_properties_dir,
            'env_models': env_models,
            'envs_centers': np.array(envs_centers).flatten().tolist()
        }]
    )


    # Add commands to the launch actions
    launch_actions.extend([set_gazebo_models_path_cmd, 
                           declare_use_cartographer_cmd, 
                           gz_server_cmd, gz_client_cmd, 
                           reset_env_cmd,
                           envs_properties_publisher_cmd])



    # Create and configure nodes for each environment
    for i in range(num_envs):
        namespace = f'env_{i}'
        env_center = envs_centers[i]
        env_model = env_models[i]
        env_model_properties_path = os.path.join(models_properties_dir, f'{env_model}_properties.json')
        rviz_config_file = utils.create_rviz_config(rviz_config_dir, namespace)

        # Calculate robot's initial pose
        if random_pose:
            robot_init_pose = utils.get_random_pose(env_model_properties_path, env_center)
        else:
            robot_init_pose = [robot_pos_x + env_center[0],
                               robot_pos_y + env_center[1],
                               robot_pos_yaw]

        # Robot state publisher node
        robot_state_pub_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static')
            ],
        )   

        # Robot spawner node
        robot_spawner_cmd = Node(
            package=package_name,
            executable='robot_spawner',
            output='screen',
            namespace=namespace,
            parameters=[{
                'robot_name': 'tb3',
                'robot_namespace': namespace,
                'robot_urdf_path': robot_urdf_path,
                'x': float(robot_init_pose[0]),
                'y': float(robot_init_pose[1]),
                'z': 0.01,
                'yaw': float(robot_init_pose[2]),
                'env_center': env_center,

                # Parameters needed for reset_environment
                'env_model_properties_path': env_model_properties_path, 
                'cartographer_config_path': os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'config'),
                'cartographer_config_basename': 'turtlebot3_lds_2d.lua',
                'rviz_config_path': rviz_config_file,
            }]
        )

        # Cartographer SLAM node
        cartographer_cmd = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '-configuration_directory', os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'config'),
                '-configuration_basename', 'turtlebot3_lds_2d.lua',
            ],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static')
            ],
            condition=IfCondition(LaunchConfiguration('use_cartographer')),
        )

        # RViz node
        rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=['-d', rviz_config_file],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static')
            ],
            condition=IfCondition(LaunchConfiguration('use_cartographer')),
        )

        # Occupancy grid publisher node
        occupancy_grid_cmd = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '-resolution', str(map_resolution),
                '-publish_period_sec', str(map_publish_period),
            ],
            condition=IfCondition(LaunchConfiguration('use_cartographer')),
        )




        # Add nodes to launch actions
        launch_actions.extend([robot_state_pub_cmd, robot_spawner_cmd, cartographer_cmd, rviz_cmd, occupancy_grid_cmd])
    


    return LaunchDescription(launch_actions)
