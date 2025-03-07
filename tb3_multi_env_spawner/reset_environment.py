import rclpy
import os
import signal
import subprocess
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import GetModelList
from rcl_interfaces.srv import GetParameters
from custom_interfaces.srv import ResetEnv


from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan



from tb3_multi_env_spawner import utils




class ResetEnvironment(Node):
    """Service to reset simulation environments."""

    def __init__(self) -> None:
        super().__init__('reset_environment')
        self.package_name = 'tb3_multi_env_spawner'
        self._reset_params = {}
        self._sensor_data = {
            'tf': None,
            'map': None,
            'scan': None
        }
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self._init_service()
        self.get_logger().info('Reset Environment node initialized')


    def _init_service(self) -> None:
        """ Initializes Ros services and clients """
        self.reset_service = self.create_service(
            ResetEnv, 
            'reset_environment', 
            self._reset_service_callback,
            callback_group=self.cb_group
        )

        self.delete_entity_client = self.create_client(
            DeleteEntity,
            'delete_entity',
        )
        self._wait_for_service(self.delete_entity_client, 'delete_entity')

        self.model_list_client = self.create_client(
            GetModelList,
            'get_model_list'
        )
        self._wait_for_service(self.model_list_client, 'get_model_list')


    def _reset_service_callback(self, request, response) -> ResetEnv.Response:
        """Reset simulation environment."""
        try:
            self.get_logger().info(f'Resetting environment {request.entity_namespace}')
            # Get parameters client:
            param_client = self.create_client(
                GetParameters,
                f'/{request.entity_namespace}/robot_spawner/get_parameters'
            )
            self._wait_for_service(param_client, 'get_parameters')

            # Execute reset sequence
            self._get_parameters(param_client)
            self._delete_existing_entity(request)
            self._cleanup_processes(request.entity_namespace)
            self._spawn_new_entity(request.entity_namespace)
            self._restart_mapping_nodes(request.entity_namespace)
            self._verify_sensor_ready()
            self._verify_model_is_spawned()

            response.success = True

        except Exception as e:
            self.get_logger().error(f'Error resetting environment: {e}')
            response.success = False

        return response

    def _get_parameters(self, client) -> None:
        """ Get parameters of the environment to be reset """
        param_names = [
            "robot_name", "robot_namespace", "robot_urdf_path", "use_namespace",
            "env_model_properties_path", "env_center", "cartographer_config_path",
            "cartographer_config_basename", "rviz_config_path"
        ]

        future = client.call_async(GetParameters.Request(names=param_names))
        self._wait_for_future(future)
        self._reset_params = self._parse_parameters(param_names, future.result())
        self.get_logger().info(f'Parameters retrieved')

    def _parse_parameters(self, names, response) -> dict:
        """ Convert parameters to dictionary """
        result = {}
        for name, value in zip(names, response.values):
            if value.type == 1:  # PARAMETER_BOOL
                result[name] = value.bool_value
            elif value.type == 2:  # PARAMETER_INTEGER
                result[name] = value.integer_value
            elif value.type == 3:  # PARAMETER_DOUBLE
                result[name] = value.double_value
            elif value.type == 4:  # PARAMETER_STRING
                result[name] = value.string_value
            elif value.type == 5:  # PARAMETER_BYTE_ARRAY
                result[name] = list(value.byte_array_value)
            elif value.type == 6:  # PARAMETER_BOOL_ARRAY
                result[name] = list(value.bool_array_value)
            elif value.type == 7:  # PARAMETER_INTEGER_ARRAY
                result[name] = list(value.integer_array_value)
            elif value.type == 8:  # PARAMETER_DOUBLE_ARRAY
                result[name] = list(value.double_array_value)
            elif value.type == 9:  # PARAMETER_STRING_ARRAY
                result[name] = list(value.string_array_value)
            else:
                result[name] = None  # Unknown type
        return result
    
    def _delete_existing_entity(self, request) -> None:
        """ Delete existing entity """
        req = DeleteEntity.Request()
        req.name = f'{request.entity_namespace}_{request.entity_name}'
        future = self.delete_entity_client.call_async(req)
        self._wait_for_future(future)
        if future.result().success:
            self.get_logger().info(f'{future.result().status_message}')
        else:
            self.get_logger().error(f'{future.result().status_message}')

    def _cleanup_processes(self, namespace) -> None:
        """ Kill all processes in the entity namespace """
        target_processes = ['cartographer', 'robot_spawner']
        pids = self._find_processes(target_processes, namespace)
        self._terminate_processes(pids)
        self.get_logger().info(f'Processes terminated')

    def _find_processes(self, target_processes, namespace) -> list:
        """ Find all processes in the entity namespace """
        pids = []
        for proc in target_processes:
            try:
                result = subprocess.check_output(
                    ['pgrep', '-f', f'{proc}.*--ros-args.*__ns:=/{namespace}']
                )
                pids.extend(map(int, result.decode().split()))
            except subprocess.CalledProcessError:
                self.get_logger().info(f'No {proc} process found in {namespace}')
        return pids

    def _terminate_processes(self, pids) -> None:
        """ Terminate all processes """
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                time.sleep(0.5)
            except OSError as e:
                self.get_logger().error(f'Error terminating process {pid}: {e}')


    def _spawn_new_entity(self, namespace) -> None:
        """ Spawn new entity """
        spawn_position = utils.get_random_pose(
            self._reset_params['env_model_properties_path'],
            self._reset_params['env_center']
        )

        command = [
            'ros2', 'run', self.package_name, 'robot_spawner',
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-p', f'robot_name:={self._reset_params["robot_name"]}',
            '-p', f'robot_namespace:={namespace}',
            '-p', f'robot_urdf_path:={self._reset_params["robot_urdf_path"]}',
            '-p', f'x:={float(spawn_position[0])}',
            '-p', f'y:={float(spawn_position[1])}',
            '-p', f'z:={0.01}',
            '-p', f'yaw:={float(spawn_position[2])}',
            '-p', f'env_model_properties_path:={self._reset_params["env_model_properties_path"]}',
            '-p', f'env_center:={self._reset_params["env_center"]}',
            '-p', f'cartographer_config_path:={self._reset_params["cartographer_config_path"]}',
            '-p', f'cartographer_config_basename:={self._reset_params["cartographer_config_basename"]}',
            '-p', f'rviz_config_path:={self._reset_params["rviz_config_path"]}',
        ]
        self._execute_command(command)

    def _restart_mapping_nodes(self, namespace) -> None:
        """ Restart mapping nodes """
        self._start_cartographer(namespace)
        self._start_occupancy_grid(namespace)
        self._init_sensor_subscription(namespace)
        self.get_logger().info('Mapping nodes restarted')



    def _start_cartographer(self, namespace) -> None:
        """Launch cartographer node with namespace configuration."""
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', self._reset_params['cartographer_config_path'],
            '-configuration_basename', self._reset_params['cartographer_config_basename'],
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-r', f'/tf:=/{namespace}/tf',
            '-r', f'/tf_static:=/{namespace}/tf_static',
            '-p', 'use_sim_time:=True'
        ]
        self._execute_command(command)


    def _start_occupancy_grid(self, namespace) -> None:
        """Launch occupancy grid node."""
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-p', 'use_sim_time:=True',
            '-p', 'resolution:=0.05',
            '-p', 'publish_period_sec:=1.0'
        ]
        self._execute_command(command)


    def _init_sensor_subscription(self, namespace) -> None:
        """Initialize sensor data subscriptions."""
        namespace = self._reset_params["robot_namespace"]
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            f'/{namespace}/tf',
            lambda msg: self._sensor_data.update(tf=msg),
            10,
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{namespace}/map',
            lambda msg: self._sensor_data.update(map=msg),
            10,
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{namespace}/scan',
            lambda msg: self._sensor_data.update(scan=msg),
            10,
        )

    def _verify_model_is_spawned(self) -> None:
        """Verify that the model has been spawned."""
        model_name = f'{self._reset_params["robot_namespace"]}_{self._reset_params["robot_name"]}'
        future = self.model_list_client.call_async(GetModelList.Request())
        self._wait_for_future(future)
        if model_name not in future.result().model_names:
            self.get_logger().info(f"Waiting for {model_name} to spawn...")
            self._verify_model_is_spawned()
        else: 
            self.get_logger().info(f"{model_name} spawned successfully")




    def _wait_for_service(self, client, service_name) -> None:
        """Wait for service to be available."""
        self.get_logger().info(f'Waiting for {service_name} service...')
        while not client.wait_for_service(timeout_sec=0.01):
            pass
        self.get_logger().info(f'...{service_name} connected!')


    def _wait_for_future(self, future, timeout=10):
        """Wait for future completion with timeout."""
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                raise TimeoutError("Service call timed out")
            time.sleep(0.1)


    def _verify_sensor_ready(self) -> None:
        """Confirm all sensor topics are active."""
        self.get_logger().info("Verifying sensor readiness...")
        while None in self._sensor_data.values():
            time.sleep(0.1)
        self.get_logger().info("All sensors operational")


    def _execute_command(self, command) -> subprocess.Popen:
        """Execute system command with error handling."""
        try:
            return subprocess.Popen(command)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command execution failed: {str(e)}")
            return None





def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetEnvironment()
    
    executor = MultiThreadedExecutor()
    executor.add_node(reset_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        reset_node._logger_info("Shutting down...")
    finally:
        executor.remove_node(reset_node)
        reset_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()