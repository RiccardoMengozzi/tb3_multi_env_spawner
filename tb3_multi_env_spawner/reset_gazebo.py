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
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters




from tb3_multi_env_spawner import utils




class ResetGazebo(Node):
    """Service to reset simulation environments."""

    def __init__(self) -> None:
        super().__init__('reset_gazebo')
        self.package_name = 'tb3_multi_env_spawner'

        # Declare parameters for robot spawn settings
        self.declare_parameter('num_envs', 1)  # Number of environments
        self.declare_parameter('robot_name', 'dummy_robot')  # Name of the robot
        self.declare_parameter('cartographer_config_path', 'dummy_config.lua')  # Path to cartographer config
        self.declare_parameter('cartographer_config_basename', 'dummy_config.lua')  # Cartographer config basename
        self.declare_parameter('map_resolution', 0.05)  # Resolution of the occupancy grid
        self.declare_parameter('map_publish_period', 1.0)  # Publish period of the occupancy grid

        # Retrieve the parameter values after declaration
        self.num_envs = self.get_parameter('num_envs').get_parameter_value().integer_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.cartographer_config_path = self.get_parameter('cartographer_config_path').get_parameter_value().string_value
        self.cartographer_config_basename = self.get_parameter('cartographer_config_basename').get_parameter_value().string_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_publish_period = self.get_parameter('map_publish_period').get_parameter_value().double_value

        # GAZEBO
        self.declare_parameter('world_path', 'dummy.world') 
        self.declare_parameter('gui', True) 
        self.declare_parameter('verbose', False) 


        self.world_path = self.get_parameter('world_path').get_parameter_value().string_value
        self.gui = self.get_parameter('gui').get_parameter_value().bool_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value




        self.cb_group = ReentrantCallbackGroup()

        self.reset_gazebo_service = self.create_service(
            Trigger,
            '/reset_gazebo_service',
            self._reset_gazebo_service_callback,
            callback_group=self.cb_group
        )

        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.gazebo_param_client = self.create_client(GetParameters, '/gazebo/get_parameters')


    def _wait_for_service(self, client, service_name) -> None:
        self.get_logger().info(f'Waiting for {service_name} service...')
        while not client.wait_for_service(timeout_sec=5.0):
            print(f'...waiting for {service_name} service...')
            time.sleep(1)
            pass
        self.get_logger().info(f'...{service_name} connected!')

    def _wait_for_service_to_be_inactive(self, client, service_name) -> None:
        self.get_logger().info(f'Waiting for {service_name} service to be inactive...')
        while client.wait_for_service(timeout_sec=5.0):
            print(f'...waiting for {service_name} service to be inactive...')
            time.sleep(1)
            pass
        self.get_logger().info(f'...{service_name} inactive!')



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
            return proc
        except Exception as e:
            self.get_logger().error(f"Command failed: {str(e)}")
            raise

    def _find_processes(self, target_processes) -> list:
        """ Find all processes in the entity namespace """
        pids = []
        for proc in target_processes:
            try:
                result = subprocess.check_output(
                    ['pgrep', '-f', f'{proc}.*']
                )
                pids.extend(map(int, result.decode().split()))
            except subprocess.CalledProcessError:
                self.get_logger().info(f'No {proc} process found')
        return pids

    def _terminate_processes(self, pids) -> None:
        """Forcefully terminates Gazebo processes and their entire hierarchy.
        
        Args:
            pids (list): List of Gazebo process IDs (gzserver/gzclient)
        """
        # Try graceful termination first
        for pid in pids:
            try:
                process = psutil.Process(pid)
                children = process.children(recursive=True)
                
                # Send SIGTERM to entire process tree
                for child in children:
                    child.terminate()
                process.terminate()
                
            except psutil.NoSuchProcess:
                continue


        # Force kill any survivors
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



    def _reset_gazebo_service_callback(self, request, response):
        try:
            self.get_logger().info("Resetting Gazebo...")

            [self._delete_existing_entity(namespace=f'env_{i}') for i in range(self.num_envs)]
            self._restart_gazebo()
            [self._spawn_new_entity(namespace=f'env_{i}') for i in range(self.num_envs)]



            response.success = True
        except Exception as e:
            self.get_logger().error(
                f"Error resetting gazebo:\n{traceback.format_exc()}"
            )
            response.success = False

        return response
    
    def _delete_existing_entity(self, namespace) -> None:
        
        self._wait_for_service(self.delete_entity_client, 'delete_entity')

        req = DeleteEntity.Request()
        req.name = f'{namespace}_{self.robot_name}'
        future = self.delete_entity_client.call_async(req)
        self._wait_for_future(future)     
        if future.result().success:
            self.get_logger().info(future.result().status_message)
            future.cancel()
            del future
        else:
            raise Exception(future.result().status_message)
        
    def _restart_gazebo(self) -> None:
        """ Restart Gazebo """
        self.get_logger().info("Restarting Gazebo...")
        self._kill_gazebo_and_cartographer()
        # Verify gazebo node does not exist
        self._wait_for_service_to_be_inactive(self.gazebo_param_client, 'gazebo/get_parameters')
        self._start_gazebo()
        # Veify gazebo node exists
        self._wait_for_service(self.gazebo_param_client, 'gazebo/get_parameters')
        [self._start_cartographer(namespace=f'env_{i}') for i in range(self.num_envs)]


    def _kill_gazebo_and_cartographer(self) -> None:
        target_processes = ['gzserver', 'gzclient', 'cartographer']
        pids = self._find_processes(target_processes)
        self._terminate_processes(pids)

    def _start_gazebo(self) -> None:
        command = [
            'ros2', 'launch', 'gazebo_ros', 'gzserver.launch.py',
            f'world:={self.world_path}',
            f'verbose:={self.verbose}',
        ]
        process = self._execute_command(command)

        if self.gui:
            command = [
                'ros2', 'launch', 'gazebo_ros', 'gzclient.launch.py',
                f'verbose:={self.verbose}',
            ]
            process = self._execute_command(command)


    def _start_cartographer(self, namespace) -> None:
        """Launch cartographer node with namespace configuration."""
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', self.cartographer_config_path,
            '-configuration_basename', self.cartographer_config_basename,
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-r', f'/tf:=/{namespace}/tf',
            '-r', f'/tf_static:=/{namespace}/tf_static',
            '-p', 'use_sim_time:=True'
        ]
        process = self._execute_command(command)

        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
            '-resolution', str(self.map_resolution),
            '-publish_period_sec', str(self.map_publish_period),
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-p', 'use_sim_time:=True',
        ]
        process = self._execute_command(command)


    def _spawn_new_entity(self, namespace) -> None:
        """ Spawn new entity """
        self.get_logger().info(f'Spawning robot {self.robot_name} in {namespace}')
        spawn_robot_client = self.create_client(Trigger, f'/{namespace}/spawn_robot')
        self._wait_for_service(spawn_robot_client, f'/{namespace}/spawn_robot')

        req = Trigger.Request()
        future = spawn_robot_client.call_async(req)
        self._wait_for_future(future)     
        if future.result().success:
            self.get_logger().info(future.result().message)
            future.cancel()
            del future
        else:
            raise Exception(future.result().message)

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetGazebo()
    
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