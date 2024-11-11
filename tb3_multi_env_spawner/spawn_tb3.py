
import xml.etree.ElementTree as ET
from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.node import Node


class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')

        # Declare parameters
        self.declare_parameter('robot_urdf', 'dummy.urdf')
        self.declare_parameter('robot_name', 'dummy_robot')
        self.declare_parameter('robot_namespace', 'dummy_robot_ns')
        self.declare_parameter('use_namespace', True)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)

        # Get parameters
        self.robot_urdf = self.get_parameter('robot_urdf').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.use_namespace = self.get_parameter('use_namespace').get_parameter_value().bool_value
        self.position_x = self.get_parameter('x').get_parameter_value().double_value
        self.position_y = self.get_parameter('y').get_parameter_value().double_value
        self.position_z = self.get_parameter('z').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value

        # Set up client
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info('Connecting to `/spawn_entity` service...')
        self.client.wait_for_service()
        self.get_logger().info('...connected!')

        # Modify URDF and call spawn service
        self.modify_urdf_with_namespace()
        self.call_spawn_service()

    def modify_urdf_with_namespace(self):
        """Modify the URDF with namespace remappings if applicable."""
        try:
            tree = ET.parse(self.robot_urdf)
            root = tree.getroot()

            for plugin in root.iter('plugin'):
                plugin_type = plugin.attrib.get('name', '')
                ros_element = plugin.find('ros')                    
                if 'turtlebot3_diff_drive' in plugin_type:
                    self.add_namespace_and_remap(ros_element, 'diff_drive_plugin')
                elif 'turtlebot3_imu' in plugin_type:
                    self.add_namespace_and_remap(ros_element, 'imu_plugin')
                elif 'turtlebot3_joint_state' in plugin_type:
                    self.add_namespace_and_remap(ros_element, 'joint_state_plugin')

            self.modified_urdf = ET.tostring(root, encoding='unicode')
        except Exception as e:
            self.get_logger().error(f"Failed to modify URDF: {e}")
            raise

    def add_namespace_and_remap(self, ros_element, plugin_name):
        """Add namespace and TF remappings to URDF plugins."""
        namespace_tag = ET.SubElement(ros_element, 'namespace')
        namespace_tag.text = f'/{self.robot_namespace}'
        remap_tf = ET.SubElement(ros_element, 'remapping')
        remap_tf.text = f'/tf:=/{self.robot_namespace}/tf'
        remap_tf_static = ET.SubElement(ros_element, 'remapping')
        remap_tf_static.text = f'/tf_static:=/{self.robot_namespace}/tf_static'
        self.get_logger().info(f"Modified {plugin_name} with namespace and remappings.")

    def call_spawn_service(self):
        """Set up the service request and call the spawn entity service."""
        request = SpawnEntity.Request()
        request.name = self.robot_name
        request.xml = self.modified_urdf
        request.robot_namespace = self.robot_namespace if self.use_namespace else ''
        request.initial_pose.position.x = self.position_x
        request.initial_pose.position.y = self.position_y
        request.initial_pose.position.z = self.position_z
        request.initial_pose.orientation.z = self.yaw

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned robot `{self.robot_name}` successfully.")
        else:
            self.get_logger().error(f"Failed to spawn robot `{self.robot_name}`: {future.exception()}")

    def shutdown(self):
        self.get_logger().info('Shutting down RobotSpawner node.')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    robot_spawner = RobotSpawner()

    try:
        rclpy.spin(robot_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        robot_spawner.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
