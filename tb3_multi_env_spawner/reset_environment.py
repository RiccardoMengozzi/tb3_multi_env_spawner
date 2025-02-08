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
from utils import utils


class ResetEnvironmentService(Node):
    def __init__(self):
        super().__init__('reset_environment')
        self.package_name = 'tb3_multi_env_spawner'


        self.callback_group = ReentrantCallbackGroup()
        self.done_event = Event()
        # Create the custom service using the ResetEnvironment type
        self.srv = self.create_service(ResetEnvironment, 
                                       'reset_environment', 
                                       self.reset_environment_callback,
                                       callback_group=self.callback_group)
        
        self.del_entity_cli = self.create_client(DeleteEntity, 
                                                'delete_entity', 
                                                callback_group=self.callback_group)
        
        self.get_logger().info('connecting to delete_entity service...')
        while not self.del_entity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('connecting to delete_entity service...')
        self.get_logger().info('...connected!')
        

    def send_delete_entity_request(self, request):
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
        for pid in pids:
            try:
                # Send SIGTERM to the process
                os.kill(pid, signal.SIGTERM)
                self.get_logger().info(f'[simulation_reset] Successfully sent SIGTERM to PID {pid}.')
                
                # Wait for the process to terminate
                time.sleep(0.5)  # Give it a moment to shut down
                # if os.path.exists(f'/proc/{pid}'):
                #     # self.get_logger().error(f'[simulation_reset] Process {pid} is still running, sending SIGKILL.')
                #     os.kill(pid, signal.SIGKILL)  # Force kill if it's still running
            except OSError as e:
                self.get_logger().error(f'[simulation_reset] Error killing process {pid}: {e}')

    



    def reset_environment_callback(self, request, response):
        try:
            service_name = f'/{request.entity_namespace}/robot_spawner/get_parameters'
            self.params_cli = self.create_client(GetParameters,
                                                service_name,
                                                callback_group=self.callback_group)
            
            self.get_logger().info(f'connecting to {service_name} service...')
            while not self.del_entity_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'connecting to {service_name} service...')
            self.get_logger().info('...connected!')

            names = [
                "robot_name", "robot_namespace", "robot_urdf_path", "use_namespace", "env_model_properties_path", 
                "env_center", "cartographer_config_path", "cartographer_config_basename", "rviz_config_path"
            ]
            get_parameters_response = self.send_get_parameters_request(names)
            self.params = self.create_parameters_dictionary(names, get_parameters_response)

            ### Delete entity
            delete_entity_response = self.send_delete_entity_request(request)
            response.success = delete_entity_response.success
            self.get_logger().info(f'{delete_entity_response.status_message}')

            ### shutdown cartographer and robot_spawner
            pids = self.find_process_pids(['cartographer', 'robot_spawner'], namespace=request.entity_namespace)
            self.kill_processes(pids)

            ### Spawn entity
            # get random availiable intial pose
            robot_init_pose = utils.get_random_pose(self.params["env_model_properties_path"], self.params["env_center"])
            print("ENV CENTER: ", self.params["env_center"])
            command = [
                'ros2', 'run', self.package_name, 'robot_spawner',
                '--ros-args',
                '-r', '__ns:=/' + request.entity_namespace,
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
            # Execute the command

            try:
                subprocess.Popen(command)
            except subprocess.CalledProcessError as e:
                print(f"Error while running the command: {e}")

            ### Start cartographer
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
            # Execute the command
            try:
                subprocess.Popen(command)
            except subprocess.CalledProcessError as e:
                print(f"Error while running the command: {e}")



            ### Start cartographer occupancy grid node
            command = [
                'ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
                '--ros-args',
                '-r', f'__ns:=/{self.params["robot_namespace"]}',
                '-p', 'use_sim_time:=True',
                '-p', 'resolution:=0.05',
                '-p', 'publish_period_sec:=1.0'
            ]
            # Execute the command
            try:
                subprocess.Popen(command)
            except subprocess.CalledProcessError as e:
                print(f"Error while running the command: {e}")

        except Exception as e:
            self.get_logger().error(f"error: {e}")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)

    reset_env = ResetEnvironmentService()

    executor = MultiThreadedExecutor()
    rclpy.spin(reset_env, executor)
    executor.shutdown()
    reset_env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
