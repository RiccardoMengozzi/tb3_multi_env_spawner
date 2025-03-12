import rclpy
import os
import signal
import subprocess
import time
import traceback
import psutil
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import GetModelList
from std_srvs.srv import Trigger

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan



from tb3_multi_env_spawner import utils




class ResetEnvironment(Node):
    """Service to reset simulation environments."""

    def __init__(self) -> None:
        super().__init__('reset_environment')
        self.package_name = 'tb3_multi_env_spawner'
        self.active_processes = []

        # Declare parameters for robot spawn settings
        self.declare_parameter('robot_urdf_path', 'dummy.urdf')  # Path to the URDF file
        self.declare_parameter('robot_name', 'dummy_robot')  # Name of the robot
        self.declare_parameter('robot_namespace', 'dummy_robot_ns')  # Namespace for the robot
        self.declare_parameter('env_model_properties_path', 'dummy_world.json')  # Path to environment properties
        self.declare_parameter('cartographer_config_path', 'dummy_config.lua')  # Path to cartographer config
        self.declare_parameter('cartographer_config_basename', 'dummy_config.lua')  # Cartographer config basename
        self.declare_parameter('env_center', [0, 0])  # Center of the environment in the world
        self.declare_parameter('rviz_config_path', 'dummy_config.rviz')  # Path to RViz configuration
        self.declare_parameter('map_resolution', 0.05)  # Resolution of the occupancy grid
        self.declare_parameter('map_publish_period', 1.0)  # Publish period of the occupancy grid

        # Retrieve the parameter values after declaration
        self.robot_urdf = self.get_parameter('robot_urdf_path').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.env_model_properties_path = self.get_parameter('env_model_properties_path').get_parameter_value().string_value
        self.cartographer_config_path = self.get_parameter('cartographer_config_path').get_parameter_value().string_value
        self.cartographer_config_basename = self.get_parameter('cartographer_config_basename').get_parameter_value().string_value
        self.env_center = self.get_parameter('env_center').get_parameter_value().integer_array_value
        self.rviz_config_path = self.get_parameter('rviz_config_path').get_parameter_value().string_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_publish_period = self.get_parameter('map_publish_period').get_parameter_value().double_value

        self.mutex_cb_group = MutuallyExclusiveCallbackGroup()
        self.reent_cb_group = ReentrantCallbackGroup()
        self.reset_service = self.create_service(Trigger, f'/{self.namespace}/reset_environment', self._reset_service_callback, callback_group=self.reent_cb_group)

        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.spawn_robot_client = self.create_client(Trigger, f'/{self.namespace}/spawn_robot')

        self.tf_sub = self.create_subscription(TFMessage, f'/{self.namespace}/tf', self._tf_callback, 10, callback_group=self.reent_cb_group)
        self.map_sub = self.create_subscription(OccupancyGrid, f'/{self.namespace}/map', self._map_callback, 10, callback_group=self.reent_cb_group)        
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.namespace}/scan', self._scan_callback, 10, callback_group=self.reent_cb_group)

        self.tf_msg = None
        self.map_msg = None
        self.scan_msg = None


    def _tf_callback(self, msg):
        self.tf_msg = msg

    def _map_callback(self, msg):
        self.map_msg = msg

    def _scan_callback(self, msg):
        self.scan_msg = msg

    def _wait_for_service(self, client, service_name) -> None:
        self.get_logger().info(f'Waiting for {service_name} service...')
        while not client.wait_for_service(timeout_sec=5.0):
            print(f'...waiting for {service_name} service...')
            pass
        self.get_logger().info(f'...{service_name} connected!')


    def _wait_for_future(self, future) -> None:
        while not future.done():
            # self.get_logger().info('Waiting for future...')
            time.sleep(0.5) 
        

    def _execute_command(self, command) -> subprocess.Popen:
        """Track spawned processes and redirect output"""
        try:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.active_processes.append(proc)
            return proc
        except Exception as e:
            self.get_logger().error(f"Command failed: {str(e)}")
            raise



    def _reset_service_callback(self, request, response):
        try:
            
            self.get_logger().info(f'Resetting environment {self.namespace}')
            self.cleanup_resources()
            self.tf_msg = None
            self.map_msg = None
            self.scan_msg = None

            # State machine
            self._delete_existing_entity()  
            self._cleanup_processes() 
            self._spawn_new_entity()  
            self._restart_mapping_nodes() 
            self._verify_sensor_ready() 

            response.success = True
        except Exception as e:
            self.get_logger().error(
                f"Error resetting environment:\n{traceback.format_exc()}"
            )
            response.success = False

        return response
    
    def cleanup_resources(self) -> None:   
        """ Kill all processes started by this node """
        for proc in self.active_processes:
            proc.terminate()
        self.active_processes.clear()  # Ensure all processes are removed
        self.get_logger().info(f'Cleaned resources')
    
    def _delete_existing_entity(self) -> None:
        self._wait_for_service(self.delete_entity_client, 'delete_entity')

        req = DeleteEntity.Request()
        req.name = f'{self.namespace}_{self.robot_name}'
        future = self.delete_entity_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self._wait_for_future(future)     
        if future.result().success:
            self.get_logger().info(future.result().status_message)
            future.cancel()
            del future
        else:
            raise Exception(future.result().status_message)
        

    def _cleanup_processes(self) -> None:
        """ Kill all processes in the entity namespace """
        target_processes = ['cartographer']
        pids = self._find_processes(target_processes)
        self._terminate_processes(pids)
        self.get_logger().info(f'Processes terminated')

    def _find_processes(self, target_processes) -> list:
        """ Find all processes in the entity namespace """
        pids = []
        for proc in target_processes:
            try:
                result = subprocess.check_output(
                    ['pgrep', '-f', f'{proc}.*--ros-args.*__ns:=/{self.namespace}']
                )
                pids.extend(map(int, result.decode().split()))
            except subprocess.CalledProcessError:
                self.get_logger().info(f'No {proc} process found in {self.namespace}')
        return pids

    def _terminate_processes(self, pids) -> None:
        """ Terminate all processes """
        for pid in pids:
            try:
                process = psutil.Process(pid)
                children = process.children(recursive=True)
                
                # Send SIGKILL to entire process tree
                for child in children:
                    child.send_signal(signal.SIGKILL)
                process.send_signal(signal.SIGKILL)
                
            except psutil.NoSuchProcess:
                continue


    def _spawn_new_entity(self) -> None:
        """ Spawn new entity """
        self.get_logger().info(f'Spawning robot {self.robot_name} in {self.namespace}')
        self._wait_for_service(self.spawn_robot_client, 'spawn_robot')

        req = Trigger.Request()
        future = self.spawn_robot_client.call_async(req)
        self._wait_for_future(future)     
        if future.result().success:
            self.get_logger().info(future.result().message)
            future.cancel()
            del future
        else:
            raise Exception(future.result().message)

    def _restart_mapping_nodes(self) -> None:
        """ Restart mapping nodes """
        self._start_cartographer()
        self._start_occupancy_grid()



    def _start_cartographer(self) -> None:
        """Launch cartographer node with namespace configuration."""
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', self.cartographer_config_path,
            '-configuration_basename', self.cartographer_config_basename,
            '--ros-args',
            '-r', f'__ns:=/{self.namespace}',
            '-r', f'/tf:=/{self.namespace}/tf',
            '-r', f'/tf_static:=/{self.namespace}/tf_static',
            '-p', 'use_sim_time:=True'
        ]
        process = self._execute_command(command)


    def _start_occupancy_grid(self) -> None:
        """Launch occupancy grid node."""
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
            '-resolution', str(self.map_resolution),
            '-publish_period_sec', str(self.map_publish_period),
            '--ros-args',
            '-r', f'__ns:=/{self.namespace}',
            '-p', 'use_sim_time:=True',
        ]
        process = self._execute_command(command)



    def _verify_sensor_ready(self) -> None:
        """Confirm all sensor topics are active."""
        self.get_logger().info("Verifying sensor readiness...")
        while not self.tf_msg or not self.map_msg or not self.scan_msg:
            time.sleep(0.5)
        self.get_logger().info("All sensors operational")




def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetEnvironment()
    
    executor = MultiThreadedExecutor()
    executor.add_node(reset_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        reset_node.get_logger().info("Shutting down...")
    finally:
        executor.remove_node(reset_node)
        reset_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()