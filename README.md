# `tb3_multi_env_spawner`

The `tb3_multi_env_spawner` package is designed to help multi-environment reinforcement learning (RL) training. It enables the spawning of multiple custom environments, each containing a TurtleBot3 robot. Additionally, a Cartographer instance can be created for each robot (if needed).

>[!NOTE]
>Package has been tested on Ubuntu 22.04 | ROS2 Humble. 

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
> [!IMPORTANT]
> The package depends on the [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) package, to install it:

First install turtlebot3 dependencies:
```
apt-get update \
    && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    && rm -rf /var/lib/apt/lists/*
```

Then clone the turtlebot3_simulations repository in `<path-to-your-workspace>/src`:

```
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Finally, in `<path-to_your_workspace>` (make sure to replace `<ros-distro>`):
```
rosdep install -i --from-path src --rosdistro <ros-distro> -y
colcon build --symlink-install
```

> [!IMPORTANT]
> To use the `hospital.world` these requirements are needed:
```
docopt>=0.6.2
requests
lxml
```
create a `requirements.txt` file with those requirements and run:

```
python3 -m pip install -r requirements.txt
```

> [!IMPORTANT]
> Also, to use the `hospital.world`, `small_warehouse.world`, `small_house.world`, you need to install aws and fuel models.

To install them:
```
 python3 fuel_utility.py download \
 -m XRayMachine -m IVStand -m BloodPressureMonitor -m BPCart -m BMWCart \
 -m CGMClassic -m StorageRack -m Chair \
 -m InstrumentCart1 -m Scrubs -m PatientWheelChair \
 -m WhiteChipChair -m TrolleyBed -m SurgicalTrolley \
 -m PotatoChipChair -m VisitorKidSit -m FemaleVisitorSit \
 -m AdjTable -m MopCart3 -m MaleVisitorSit -m Drawer \
 -m OfficeChairBlack -m ElderLadyPatient -m ElderMalePatient \
 -m InstrumentCart2 -m MetalCabinet -m BedTable -m BedsideTable \
 -m AnesthesiaMachine -m TrolleyBedPatient -m Shower \
 -m SurgicalTrolleyMed -m StorageRackCovered -m KitchenSink \
 -m Toilet -m VendingMachine -m ParkingTrolleyMin -m PatientFSit \
 -m MaleVisitorOnPhone -m FemaleVisitor -m MalePatientBed \
 -m StorageRackCoverOpen -m ParkingTrolleyMax \
 -d fuel_models --verbose
```

## Usage
>[!TIP]
>All parameters can be configured in `/config/launch_params.yaml` (see the [Parameters](#parameters) section below for details).

To launch the package, set the TurtleBot3 model in your terminal:

```bash
export TURTLEBOT3_MODEL=burger
```

```bash
ros2 launch tb3_multi_env_spawner tb3_multi_env_spawner.launch.py

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

>[!WARNING]
>`.world` files like `hospital.world` can take minutes to load in gazebo, have patience!!


## Environment Modes
>[!TIP]
>The `env.mode` parameter supports three different modes for environment setup:

1. **`single_model`**:
   - Loads a single environment model for all simulation instances.
   - The specific model used is defined by the `env.model` parameter.

2. **`multiple_models`**:
   - Allows different environments to be loaded simultaneously across instances.
   - The models are listed in `env.models`, with each instance assigned one of these models.

3. **`random_models`**:
   - Randomly selects an environment model for each instance from the `env.available_models` list.
   

