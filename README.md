# `tb3_multi_env_spawner`

The `tb3_multi_env_spawner` package is designed to help multi-environment reinforcement learning (RL) training. It enables the spawning of multiple custom environments, each containing a TurtleBot3 robot. Additionally, a Cartographer instance can be created for each robot (if needed).

## Table of Contents
- [Preview](#preview)
- [Installation](#installation)
- [Usage](#usage)
- [Parameters](#parameters)
- [Environment Modes](#environment-modes)

---

## Preview

![Gazebo view](https://github.com/user-attachments/assets/5ea8fbbe-7a17-4c3e-a8e2-ce05222d86fa)

---

## Installation

### Create a workspace

```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
```

### Clone the required repositories

```
cd src
git clone https://github.com/RiccardoMengozzi/tb3_multi_env_spawner.git
git clone https://github.com/RiccardoMengozzi/custom_interfaces.git
```

### Build and source your workspace
```
cd ~/turtlebot3_ws
colcon build
```

```
source ~/turtlebot3_ws/install/setup.bash
```

---

## Usage

### Spawn the environments
All parameters can be configured in `/config/launch_params.yaml` (see the [Parameters](#parameters) section below for details).

To launch the package, set the TurtleBot3 model in your terminal:

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
ros2 launch tb3_multi_env_spawner tb3_multi_env_spawner.launch.py

```
### Reset an environment

It is possible to reset one of the environments spawned with the `\reset_environment` service, which uses the `custom_interfaces/srv/ResetEnvironment` interface, whose structure is the followng:

```srv
string entity_name
string entity_namespace
---
bool success
```

To do that, just call the service specifying the name of the entity and the namespace of the environment to reset, for example:

```
ros2 service call reset_environment custom_interfaces/srv/ResetEnvironment "{entity_name: "tb3", entity_namespace: "env_2"}"
```


## Parameters
| Parameter                | Default Value       | Description                                                                                               |
|--------------------------|---------------------|-----------------------------------------------------------------------------------------------------------|
| `launch.verbose`         | `False`            | Controls whether detailed information is displayed during the launch.                                      |
| `launch.use_cartographer`| `False`            | Determines whether Cartographer SLAM is used. Set to `True` to enable Cartographer.                       |
| `gazebo.gui`             | `True`             | Enables the Gazebo GUI. Set to `False` to disable the GUI and improve performance during training.        |
| `gazebo.verbose`         | `False`            | Toggles detailed logging output from Gazebo.                                                              |
| `env.available_models`   | `['turtlebot3_bighouse', 'turtlebot3_world', 'small_warehouse', 'small_house', 'hospital']` | Lists all available simulation environments for the robot. If new environments are added, make sure to list them here.                                               |
| `env.num_envs`           | `4`                | Specifies the number of environments to spawn.                                                            |
| `env.mode`               | `'multiple_models'`| Specifies the mode of environment loading. Options: `single_model`, `multiple_models`, or `random_models` (See Environment Modes section later).|
| `env.model`              | `'small_warehouse'`| Defines the model to load if `mode` is set to `single_model`.                                             |
| `env.models`             | `['turtlebot3_bighouse', 'turtlebot3_world', 'small_warehouse', 'small_house']` | Lists models to load for `multiple_models` mode     |
| `robot.random_pose`      | `True`             | Enables random positioning of the robot within the environment.                                           |
| `robot.pose.x`           | `0.0`              | Initial x-coordinate of the robot when `random_pose` is `False`.                                          |
| `robot.pose.y`           | `0.0`              | Initial y-coordinate of the robot when `random_pose` is `False`.                                          |
| `robot.pose.yaw`         | `0.0`              | Initial orientation (yaw) of the robot when `random_pose` is `False`.                                     |


## Environment Modes

The `env.mode` parameter supports three different modes for environment setup:

1. **`single_model`**:
   - Loads a single environment model for all simulation instances.
   - The specific model used is defined by the `env.model` parameter.

2. **`multiple_models`**:
   - Allows different environments to be loaded simultaneously across instances.
   - The models are listed in `env.models`, with each instance assigned one of these models.

3. **`random_models`**:
   - Randomly selects an environment model for each instance from the `env.available_models` list.
   

