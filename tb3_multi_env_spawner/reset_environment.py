import rclpy
import os
import signal
import subprocess
import time
from threading import Event
import rclpy.callback_groups
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity
from rcl_interfaces.srv import GetParameters
from custom_interfaces.srv import ResetEnvironment
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tb3_multi_env_spawner import utils

class ResetEnvironmentService(Node):
    """
    ROS2 Node that provides a service to reset the simulation environment. 
    The service performs the following:
    - Deletes existing robot entities.
    - Stops associated processes (e.g., cartographer, robot spawner).
    - Spawns a new robot entity at a random position.
    - Restarts necessary processes (e.g., cartographer).
    """

    def __init__(self):
        """
        Initialize the ResetEnvironmentService Node.
        - Sets up the ROS2 service.
        - Connects to the `delete_entity` service.
        """
        super().__init__('reset_environment')
        self.package_name = 'tb3_multi_env_spawner'

        # Use a reentrant callback group to allow multiple threads to execute callbacks concurrently.
        self.callback_group = ReentrantCallbackGroup()
        self.done_event = Event()

        # Create the custom service using the ResetEnvironment type
        self.srv = self.create_service(
            ResetEnvironment, 
            'reset_environment', 
            self.reset_environment_callback,
            callback_group=self.callback_group
        )
        
        # Client for deleting entities
        self.del_entity_cli = self.create_client(
            DeleteEntity, 
            'delete_entity', 
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Connecting to delete_entity service...')
        while not self.del_entity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Connecting to delete_entity service...')
        self.get_logger().info('...Connected!')
        
    def send_delete_entity_request(self, request):
        """
        Sends a request to delete an entity from the simulation.

        Args:
            request: The request object containing entity details (namespace and name).

        Returns:
            The response of the delete entity service.
        """
        self.done_event.clear()
        event = Event()
        
        def done_callback(future):
            nonlocal event
            event.set()

        delete_entity_req = DeleteEntity.Request()
        delete_entity_req.name = f'{request.entity_namespace}_{request.entity_name}'
        future = self.del_entity_cli.call_async(delete_entity_req)
        future.add_done_callback(done_callback)

        # Wait for action to be done
        event.wait()
        return future.result()

    def send_get_parameters_request(self, names):
        """
        Sends a request to get parameters from the `robot_spawner` node.

        Args:
            names: List of parameter names to retrieve.

        Returns:
            The response containing the requested parameters.
        """
        self.done_event.clear()
        event = Event()
        
        def done_callback(future):
            nonlocal event
            event.set()

        get_parameters_req = GetParameters.Request()
        get_parameters_req.names = names

        if not self.params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GetParameters service is unavailable!")
            raise RuntimeError("GetParameters service is unavailable!")
        
        future = self.params_cli.call_async(get_parameters_req)
        future.add_done_callback(done_callback)
        # Wait for action to be done
        event.wait()
        return future.result()

    def create_parameters_dictionary(self, names, response):
        """
        Creates a dictionary of parameter names and values from the service response.

        Args:
            names: List of parameter names.
            response: The response object containing parameter values.

        Returns:
            A dictionary mapping parameter names to their values.
        """
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

    def find_process_pids(self, process_names, namespace):
        """
        Finds the PIDs of processes matching the specified names and namespace.

        Args:
            process_names: List of process names to find.
            namespace: Namespace to filter processes.

        Returns:
            A list of matching process PIDs.
        """
        pids = []
        for name in process_names:
            try:
                # Use pgrep to find PIDs of the specified processes
                output = subprocess.check_output(['pgrep', '-f', f'{name}.*--ros-args.*__ns:=/{namespace}'])
                pids.extend(map(int, output.decode().strip().split('\n')))
                self.get_logger().info(f'[simulation_reset] Found: {name}, PID = {pids[-1]}')
            except subprocess.CalledProcessError:
                self.get_logger().error(f'[simulation_reset] No process found for {name}.')
        return pids

    def kill_processes(self, pids):
        """
        Kills processes by their PIDs.

        Args:
            pids: List of process PIDs to kill.
        """
        for pid in pids:
            try:
                # Send SIGTERM to the process
                os.kill(pid, signal.SIGTERM)
                self.get_logger().info(f'[simulation_reset] Successfully sent SIGTERM to PID {pid}.')
                # Wait for the process to terminate
                time.sleep(0.5)  # Give it a moment to shut down

            except OSError as e:
                self.get_logger().error(f'[simulation_reset] Error killing process {pid}: {e}')

    def execute_command(self, command):
        """
        Executes a shell command in a subprocess.

        Args:
            command: List representing the command to execute.
        """
        try:
            process = subprocess.Popen(command)
            return process
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error while running the command: {e}")

    def spawn_entity(self, namespace):
        """
        Spawns a robot entity in the simulation at a random initial pose.

        Args:
            namespace: Namespace of the entity to spawn.
        """
        # Get random available initial pose
        robot_init_pose = utils.get_random_pose(self.params["env_model_properties_path"], self.params["env_center"])
        
        command = [
            'ros2', 'run', self.package_name, 'robot_spawner',
            '--ros-args',
            '-r', '__ns:=/' + namespace,
            '-p', f'robot_name:={self.params["robot_name"]}',
            '-p', f'robot_namespace:={self.params["robot_namespace"]}',
            '-p', f'robot_urdf_path:={self.params["robot_urdf_path"]}',
            '-p', f'x:={float(robot_init_pose[0])}',
            '-p', f'y:={float(robot_init_pose[1])}',
            '-p', f'z:={0.01}',
            '-p', f'yaw:={float(robot_init_pose[2])}',
            '-p', f'env_model_properties_path:={self.params["env_model_properties_path"]}',
            '-p', f'env_center:={self.params["env_center"]}',
            '-p', f'cartographer_config_path:={self.params["cartographer_config_path"]}',
            '-p', f'cartographer_config_basename:={self.params["cartographer_config_basename"]}',
            '-p', f'rviz_config_path:={self.params["rviz_config_path"]}',
        ]
        self.execute_command(command)


    def start_cartographer(self):
        """
        Starts the Cartographer SLAM process.
        """
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_node',
            '-configuration_directory', self.params['cartographer_config_path'],
            '-configuration_basename', self.params['cartographer_config_basename'],
            '--ros-args',
            '-r', f'__ns:=/{self.params["robot_namespace"]}',  # Namespace remapping
            '-r', f'/tf:=/{self.params["robot_namespace"]}/tf',  # Remapping '/tf'
            '-r', f'/tf_static:=/{self.params["robot_namespace"]}/tf_static',  # Remapping '/tf_static'
            '-p', 'use_sim_time:=True',  # Parameters
        ]
        self.execute_command(command)
 

    def start_occupancy_grid(self):
        """
        Starts the Cartographer occupancy grid node.
        """
        command = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
            '--ros-args',
            '-r', f'__ns:=/{self.params["robot_namespace"]}',
            '-p', 'use_sim_time:=True',
            '-p', 'resolution:=0.05',
            '-p', 'publish_period_sec:=1.0'
        ]
        self.execute_command(command)



    def reset_environment_callback(self, request, response):
        """
        Callback function for the `reset_environment` service.

        Args:
            request: Request object containing details about the environment to reset.
            response: Response object to indicate success or failure.

        Returns:
            The response object indicating the success of the operation.
        """
        try:
            # Create a client to retrieve parameters from the robot spawner
            service_name = f'/{request.entity_namespace}/robot_spawner/get_parameters'
            self.params_cli = self.create_client(
                GetParameters,
                service_name,
                callback_group=self.callback_group
            )
            
            self.get_logger().info(f'Connecting to {service_name} service...')
            while not self.del_entity_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Connecting to {service_name} service...')
            self.get_logger().info('...Connected!')

            # Retrieve parameters
            params_names = [
                "robot_name", "robot_namespace", "robot_urdf_path", "use_namespace", "env_model_properties_path", 
                "env_center", "cartographer_config_path", "cartographer_config_basename", "rviz_config_path"
            ]
            get_parameters_response = self.send_get_parameters_request(params_names)
            self.params = self.create_parameters_dictionary(params_names, get_parameters_response)

            # Delete entity
            delete_entity_response = self.send_delete_entity_request(request)
            response.success = delete_entity_response.success
            self.get_logger().info(f'{delete_entity_response.status_message}')

            # Shutdown cartographer and robot spawner
            pids = self.find_process_pids(['cartographer', 'robot_spawner'], namespace=request.entity_namespace)
            self.kill_processes(pids)

            # Spawn a new robot entity
            self.spawn_entity(request.entity_namespace)

            # Restart Cartographer
            self.start_cartographer()

            # Restart occupancy grid
            self.start_occupancy_grid()

            time.sleep(7)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False

        return response

def main(args=None):
    """
    Main entry point for the ResetEnvironmentService Node.
    Initializes and spins the node using a multithreaded executor, 
    and handles graceful shutdown on Ctrl+C.
    """
    rclpy.init(args=args)

    # Create the ResetEnvironmentService node
    reset_env = ResetEnvironmentService()

    # Create the multi-threaded executor
    executor = MultiThreadedExecutor()

    try:
        # Spin the node with the executor
        rclpy.spin(reset_env, executor)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        reset_env.get_logger().info('Shutting down the node...')
    finally:
        # Make sure to shut down and clean up the node
        executor.shutdown()
        reset_env.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
