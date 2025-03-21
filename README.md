# `tb3_multi_env_spawner`

The `tb3_multi_env_spawner` package is designed to help multi-environment reinforcement learning (RL) training. It enables the spawning of multiple custom environments, each containing a TurtleBot3 robot. Additionally, a Cartographer instance can be created for each robot (if needed).

## Table of Contents
- [Preview](#preview)
- [Installation](#installation)
- [Usage](#usage)
- [Parameters](#parameters)
- [Environment Modes](#environment-modes)
- [Models](#models)

---

## Preview

![Gazebo view](https://github.com/user-attachments/assets/5ea8fbbe-7a17-4c3e-a8e2-ce05222d86fa)

---

## Installation

### Install dependencies

Since the package has been created with the scope of using it for deep reinforcement learning exploration, it automatically opens Cartographer, so this is a dependecy that need to be installed. However, if you don't need it, simply remove it from the launch file.

**The project has been tested on Gazebo 11**

 Install Cartographer
 ```console
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
Install TurtleBot3 Packages
 ```console
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```



### Install needed repositories

Now create a ros2 workspace:

 ```console
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```

Clone the following repositories:
 ```console
git clone -b humble-devel \
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/RiccardoMengozzi/custom_interfaces
git clone https://github.com/RiccardoMengozzi/tb3_multi_env_spawner
```

Build your workspace:
```
cd ~/turtlebot3_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
. install/setup.bash
```


## Usage
All parameters can be configured in `/config/launch_params.yaml` (see the [Parameters](#parameters) section below for details).

To launch the package, set the TurtleBot3 model in your terminal:

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
ros2 launch tb3_multi_env_spawner tb3_multi_env_spawner.launch.py

```

**NOTE** : the first time you launch a new environment, it could take a while, in particular big environments such as `hospital`.


### Environment Reset

Every environment can be reset independently. This is done through the service `reset_environment`.
To reset an environment, simply call the service with the following request/response:

```console
string entity_name
string entity_namespace
---
bool success
```

Be sure the `entity_name` and `entity_namespace` match the ones you see on Gazebo. 
The service will delete the selected entity, restart the respective Rviz, Cartographer, and then respawn the entity in a random point.

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
   
## Models 

You can add more models in the `~/turtlebot3_ws/src/tb3_multi_env_spawner/models` directory. After that, you need to also add the paths to your new added models in the launch file, updating the environemnt variable `GAZEBO_MODEL_PATH`.

From this:

```console
   new_gazebo_models_paths = (
      f"{workspace_dir}/src/{package_name}/models/aws_models:"
      f"{workspace_dir}/src/{package_name}/models/fuel_models:"
      f"{workspace_dir}/src/{package_name}/models/turtlebot3_bighouse_model"
   )
```
To this:
```console
   new_gazebo_models_paths = (
      f"{workspace_dir}/src/{package_name}/models/aws_models:"
      f"{workspace_dir}/src/{package_name}/models/fuel_models:"
      f"{workspace_dir}/src/{package_name}/models/turtlebot3_bighouse_model:"
      f"{workspace_dir}/src/{package_name}/models/<your model>"
   )
```

Make sure the path points to the directory containing the model, without having any subdirectories inside of it.

To create a new environment, you need to create a new `.world` template with the new models you added. Simply follow the structure of the already existing `.world` templates in the `/worlds` directory. Also, every `.world` file needs a `world_properties.json` file that defines the property of the environment, such as its dimensions and the possible points of the environment in which the Turtlebot can spawn without colliding with obstacles (this can be improved by creating a software that automatically generates the properties). 

**NOTE**: The `hospital.world` does not currently have a `properties.json` file.

These `.json` files can be found in the `/extras` directory. Here an example:

``` console
{
  "x_width": 16,
  "y_width": 15,
  "x_start": -7,
  "y_start": -7,
  "resolution": 1,
  "grid": [
    [1,1,1,0,1,1,1,1,0,1,1,1,0],
    [1,1,1,0,1,1,1,1,1,1,1,1,0],
    [1,1,1,0,0,0,0,0,0,0,1,1,0],
    [1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,1,1,0,1,0,1,1,1,1,1,1,1],
    [1,1,1,0,1,1,1,1,1,0,0,1,1],
    [1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,1,1,0,1,1,1,1,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,0,1,1,1,1],
    [1,1,1,0,1,1,1,1,0,0,1,1,1],
    [1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,1]
  ]
}



```

1. **`x_width`**: defines the total x-length of the environment.

2. **`y_width`**: defines the total y-length of the environment.

3. **`x_start`**: defines the starting x-coordinate of the environment with respect to the center from which to consider the possible spawn positions of the Turtlebot.

4. **`y_start`**: defines the starting y-coordinate of the environment with respect to the center from which to consider the possible spawn positions of the Turtlebot.

5. **`resolution`**: defines the resolution (in meters) of the possible spawning points grid. 

6. **`grid`**: defines the possible spawning points in the 2d Map. **1** if the spawn is possible, **0** if not.

Example shown below:

![Image](https://github.com/user-attachments/assets/e8a85373-c9d7-4415-8f74-531e465aadd0)