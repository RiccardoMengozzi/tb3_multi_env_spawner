import rclpy
import json
import os 
import numpy as np
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from custom_interfaces.msg import EnvsProperties
from custom_interfaces.msg import EnvProperties

class EnvsPropertiesPublisher(Node):

    def __init__(self):

        super().__init__('envs_properties_publisher')
        # path to models properties .json files directory
        self.declare_parameter('envs_properties_dir_path', 'dummy_world.json')
        # list of strings of models names for each environment
        self.declare_parameter('env_models', [''])
        # list of environments centers
        self.declare_parameter('envs_centers', [0, 0])
        self.envs_properties_dir_path = self.get_parameter('envs_properties_dir_path').get_parameter_value().string_value
        self.env_models = self.get_parameter('env_models').get_parameter_value().string_array_value
        self.envs_centers = self.get_parameter('envs_centers').get_parameter_value().integer_array_value    

        self.envs_properties_pub = self.create_publisher(EnvsProperties, '/envs_properties', 10)

        self.data = self.load_envs_properties()
        self.create_timer(1.0, self.publish_envs_properties)

    def load_envs_properties(self):
        data = {}
        for filename in os.listdir(self.envs_properties_dir_path):
            if filename.endswith(".json"):
                file_path = os.path.join(self.envs_properties_dir_path, filename)
                with open(file_path, 'r') as file:
                    try:
                        data[filename] = json.load(file)  # Store data with filename as key
                    except json.JSONDecodeError as e:
                        print(f"Error loading {filename}: {e}")
        return data
    
    def create_env_properties(self, index, filename, properties):
        env_name = f'env_{index}'
        env_properties = EnvProperties()
        env_properties.name = env_name
        env_properties.file_path = os.path.join(self.envs_properties_dir_path, filename) + '_properties.json'
        # self.envs_centers is a 1D flattened list of an array of centers
        env_properties.center = [self.envs_centers[2*index], self.envs_centers[2*index+1]]
        env_properties.x_width = properties['x_width']
        env_properties.y_width = properties['y_width']
        env_properties.x_start = properties['x_start']
        env_properties.y_start = properties['y_start']
        env_properties.resolution = float(properties['resolution'])
        env_properties.grid_size = properties['grid_size']
        env_properties.grid = np.array(properties['grid']).flatten().tolist()
        return env_properties


    def publish_envs_properties(self):
        envs_properties_msg = EnvsProperties()
        self.counter = 0
        for env in self.env_models:
            filename = env + '_properties.json'
            if filename in self.data:
                envs_properties_msg.data.append(self.create_env_properties(self.counter, filename, self.data[filename]))
                self.counter += 1
        self.envs_properties_pub.publish(envs_properties_msg)

  

def main():
    rclpy.init()
    envs_properties_publisher = EnvsPropertiesPublisher()
    rclpy.spin(envs_properties_publisher)
    rclpy.shutdown()
    envs_properties_publisher.destroy_node()


if __name__ == '__main__':  
    main()