"""
This script defines a ROS 2 node for spawning a robot in a Gazebo simulation environment. The RobotSpawner node:

1. Reads configuration parameters related to the robot's URDF, position, and namespace.
2. Modifies the URDF file to include the specified namespace and remaps for relevant plugins.
3. Calls the `/spawn_entity` service to spawn the robot in the Gazebo environment at a specified initial pose.
4. Listens for a shutdown service that allows graceful termination of the node.

Functions:
- `__init__(self)`: Initializes the robot spawner node, declares parameters, connects to the spawn service, modifies the URDF, and spawns the robot.
- `modify_urdf_with_namespace(self)`: Modifies the URDF to add the correct namespace and remap certain plugins.
- `add_namespace_and_remap(self, ros_element, plugin_name)`: Adds namespace and remapping to a specific plugin in the URDF.
- `call_spawn_service(self)`: Calls the `/spawn_entity` service to spawn the robot in Gazebo.
- `main(args=None)`: The entry point for the script, initializes the ROS 2 system, creates the robot spawner node, and spins until shutdown.
"""

import xml.etree.ElementTree as ET
from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from tb3_multi_env_spawner import utils


class RobotSpawner(Node):
    """
    RobotSpawner is a ROS 2 node that spawns a robot in the Gazebo simulation. It listens for a shutdown service to
    allow the node to terminate gracefully.
    """

    def __init__(self):
        """
        Initializes the RobotSpawner node. This includes declaring parameters for robot configuration,
        setting up a service client to spawn the robot, and modifying the URDF with the appropriate namespace
        and remappings for plugins. Finally, it calls the spawn service to spawn the robot.

        The node also creates a shutdown service that listens for a shutdown command and cleans up when the
        shutdown occurs.
        """
        super().__init__("robot_spawner")
        self.first_time = True
        # Declare parameters for robot spawn settings
        self.declare_parameter("robot_urdf_path", "dummy.urdf")  # Path to the URDF file
        self.declare_parameter("robot_name", "dummy_robot")  # Name of the robot
        self.declare_parameter("robot_namespace", "dummy_robot_ns")  # Namespace for the robot
        self.declare_parameter("use_namespace", True)  # Whether to use namespace for robot
        self.declare_parameter("x", 0.0)  # Initial x position for the robot spawn
        self.declare_parameter("y", 0.0)  # Initial y position for the robot spawn
        self.declare_parameter("z", 0.0)  # Initial z position for the robot spawn
        self.declare_parameter("yaw", 0.0)  # Initial yaw (orientation) for the robot spawn

        self.declare_parameter("env_model_properties_path", "dummy_world.json")  # Path to environment properties
        self.declare_parameter("env_center", [0, 0])  # Center of the environment in the world

        # Retrieve the parameter values after declaration
        self.robot_urdf = (self.get_parameter("robot_urdf_path").get_parameter_value().string_value)
        self.robot_name = (self.get_parameter("robot_name").get_parameter_value().string_value)
        self.robot_namespace = (self.get_parameter("robot_namespace").get_parameter_value().string_value)
        self.use_namespace = (self.get_parameter("use_namespace").get_parameter_value().bool_value)
        self.position_x = self.get_parameter("x").get_parameter_value().double_value
        self.position_y = self.get_parameter("y").get_parameter_value().double_value
        self.position_z = self.get_parameter("z").get_parameter_value().double_value
        self.yaw = self.get_parameter("yaw").get_parameter_value().double_value
        self.env_model_properties_path = (self.get_parameter("env_model_properties_path").get_parameter_value().string_value)
        self.env_center = (self.get_parameter("env_center").get_parameter_value().integer_array_value)

        # Modify robot name to include namespace if required
        self.robot_name = (
            f"{self.robot_namespace}_{self.robot_name}"
            if self.use_namespace
            else self.robot_name
        )

        # Set up the client for the SpawnEntity service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.reent_cb = ReentrantCallbackGroup()
        self.service = self.create_service(
            Trigger,
            f"/{self.robot_namespace}/spawn_robot",
            self.spawn_robot_service_callback,
            callback_group=self.reent_cb,
        )

        # Spawn the robots the first time, then wait for service to be called.
        self.modify_urdf_with_namespace()
        self.call_spawn_service()

    def spawn_robot_service_callback(self, request, response):
        self.modify_urdf_with_namespace()
        self.call_spawn_service()
        response = Trigger.Response()
        response.success = True
        return response

    def modify_urdf_with_namespace(self):
        """
        Modifies the URDF file by adding namespace and remapping for plugins if applicable.
        This ensures that the robot's plugins are correctly linked to the specified namespace.
        """
        try:
            # Parse the URDF file
            tree = ET.parse(self.robot_urdf)
            root = tree.getroot()

            # Iterate over all 'plugin' elements in the URDF
            for plugin in root.iter("plugin"):
                plugin_type = plugin.attrib.get("name", "")  # Get the plugin name
                ros_element = plugin.find("ros")  # Find the 'ros' element in the plugin

                # Add namespace and remapping for specific plugins (e.g., diff_drive, imu)
                if "turtlebot3_diff_drive" in plugin_type:
                    self.add_namespace_and_remap(ros_element, "diff_drive_plugin")
                elif "turtlebot3_imu" in plugin_type:
                    self.add_namespace_and_remap(ros_element, "imu_plugin")
                elif "turtlebot3_joint_state" in plugin_type:
                    self.add_namespace_and_remap(ros_element, "joint_state_plugin")

            # Convert the modified URDF back to a string
            self.modified_urdf = ET.tostring(root, encoding="unicode")
        except Exception as e:
            # Log an error if URDF modification fails
            self.get_logger().error(f"Failed to modify URDF: {e}")
            raise

    def add_namespace_and_remap(self, ros_element, plugin_name):
        """
        Adds namespace and remappings to URDF plugins for proper communication with ROS.
        This is necessary to ensure the robot's topics and services are correctly namespaced.
        """
        # Add the namespace to the plugin's ros element
        namespace_tag = ET.SubElement(ros_element, "namespace")
        namespace_tag.text = f"/{self.robot_namespace}"

        # Add remapping for tf and tf_static
        remap_tf = ET.SubElement(ros_element, "remapping")
        remap_tf.text = f"/tf:=/{self.robot_namespace}/tf"
        remap_tf_static = ET.SubElement(ros_element, "remapping")
        remap_tf_static.text = f"/tf_static:=/{self.robot_namespace}/tf_static"

        # Log that the plugin was modified
        self.get_logger().info(f"Modified {plugin_name} with namespace and remappings.")

    def call_spawn_service(self):
        """
        Set up and calls the SpawnEntity service to spawn the robot in the Gazebo simulation.
        This service request includes the robot's URDF, name, namespace, and initial pose.
        """
        # Set up the service request
        request = SpawnEntity.Request()
        request.name = self.robot_name
        request.xml = self.modified_urdf
        request.robot_namespace = self.robot_namespace if self.use_namespace else ""
        if self.first_time:
            request.initial_pose.position.x = self.position_x
            request.initial_pose.position.y = self.position_y
            request.initial_pose.position.z = self.position_z
            request.initial_pose.orientation.z = self.yaw
        else:
            random_pose = utils.get_random_pose(self.env_model_properties_path, self.env_center)
            request.initial_pose.position.x = random_pose[0]
            request.initial_pose.position.y = random_pose[1]
            request.initial_pose.position.z = self.position_z
            request.initial_pose.orientation.z = random_pose[2]

        # Call the SpawnEntity service asynchronously and wait for the result
        self.get_logger().info("Connecting to `/spawn_entity` service...")
        self.client.wait_for_service()  # Wait for the service to be available
        self.get_logger().info("...connected!")
        future = self.client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        if self.first_time:
            rclpy.spin_until_future_complete(self, future)
            self.first_time = False
        else:
            while not future.done():
                self.get_logger().info(
                    f"Waiting for service to spawn robot `{self.robot_name}`..."
                )
                time.sleep(0.5)
            
        # Log the result of the spawn service
        if future.result() is not None:
            self.get_logger().info(f"Spawned robot `{self.robot_name}` successfully.")
        else:
            self.get_logger().error(
                f"Failed to spawn robot `{self.robot_name}`: {future.exception()}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
