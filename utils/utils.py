import os
import math
import glob
import json
import numpy as np
import random
import re
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


import os
import json
import math

def generate_centers(n_points : int, env_models : list[str], models_properties_dir : str) -> list[list[int]]:
    # Load the width properties for each environment model
    x_widths = []
    y_widths = []
    for env_model in env_models:
        properties_path = os.path.join(models_properties_dir, f'{env_model}_properties.json')
        with open(properties_path, 'r') as file:
            data = json.load(file)
        x_widths.append(data['x_width'])
        y_widths.append(data['y_width'])

    # Get the maximum width and height to determine spacing for the grid
    max_x_width = max(x_widths)
    max_y_width = max(y_widths)

    # Determine grid dimensions (rows and columns) for a near-square arrangement
    cols = math.ceil(math.sqrt(n_points))
    rows = math.ceil(n_points / cols)

    # Generate center points for each environment
    points = []
    for row in range(rows):
        for col in range(cols):
            if len(points) >= n_points:
                break  # Stop if we have enough points

            # Calculate x based on column and zigzag pattern for rows
            x_offset = col * max_x_width if row % 2 == 0 else (cols - 1 - col) * max_x_width
            y_offset = row * max_y_width
            points.append((x_offset, y_offset))
    
    return points[:n_points]


def create_multi_env_world(num_envs : int, 
                           env_models : list[str],  
                           package_name : str,
                           mode='single_model') -> str:

    current_dir = os.getcwd()  
    models_properties_dir = os.path.join(current_dir, 'src', package_name, 'extras', 'models_properties')

    
        # Define the beginning of the SDF world
    try:
        sdf_start = '''<?xml version="1.0"?>
<sdf version="1.6">
    <world name="default">

        <include>
            <uri>model://ground_plane</uri>
        </include>

        <include>
            <uri>model://sun</uri>
        </include>

        <scene>
            <shadows>false</shadows>
        </scene>

        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
                <view_controller>orbit</view_controller>
                <projection_type>perspective</projection_type>
            </camera>
        </gui>

        <physics type="ode">
            <real_time_update_rate>1000.0</real_time_update_rate>
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
            <ode>
                <solver>
                    <type>quick</type>
                    <iters>150</iters>
                    <precon_iters>0</precon_iters>
                    <sor>1.400000</sor>
                    <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
                </solver>
                <constraints>
                    <cfm>0.00001</cfm>
                    <erp>0.2</erp>
                    <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
                    <contact_surface_layer>0.01000</contact_surface_layer>
                </constraints>
            </ode>
        </physics>\n'''

    # Generate the big house blocks dynamically
        house_blocks = ""
        centers = generate_centers(num_envs, env_models, models_properties_dir=models_properties_dir)
        for i in range(num_envs):
            env_model = env_models[i]
            center = centers[i]
            format_world_file(current_dir, package_name, env_model, center[0], center[1], namespace=f'env_{i}_')
            with open(os.path.join(current_dir, 'src', package_name, 'extras', 'worlds_models_text', f'{env_model}.xml'), 'r') as file:
                house_blocks += f'''
                {file.read()} \n'''
        # Define the end of the SDF world
        sdf_end = '''
    </world>
</sdf>
'''

        sdf_content = sdf_start + house_blocks + sdf_end



        save_path = os.path.join(current_dir, 'src', package_name, 'worlds')

        file_name = f"multi_env_{env_model}.world"
        if mode == 'random_models':
            file_name = "multi_env_multi_random_models.world"
        elif mode == 'multiple_models':
            file_name = f"multi_env_multi_models.world"
        file_path = os.path.join(save_path, file_name)

        with open(file_path, "w") as f:
            f.write(sdf_content)

        return file_name
    
    except Exception as e:
        print(f"[ERROR] [utils]: {e}")
        return None
    

def get_random_pose(env_model_path : str, env_center : int) -> list[int]:
    with open(env_model_path, 'r') as file:
        data = json.load(file)
    # Access the starting coordinates
    x_start = data['x_start']
    y_start = data['y_start']
    resolution = data['resolution']

    # Access the grid and convert it to a numpy array
    grid = np.array(data['grid'])
    # Find the indices of the cells where the value is 1
    indices = np.argwhere(grid == 1)
    # Select a random coordinate from the indices
    random_index = random.choice(indices)

    # Adjust the coordinates based on x_start and y_start

    random_coordinate = (x_start + random_index[0] * resolution, y_start + random_index[1] * resolution)  # (x, y)
    random_yaw = np.random.uniform(0, 2*np.pi)
    random_pose = [random_coordinate[0] + env_center[0], random_coordinate[1] + env_center[1], random_yaw]


    return random_pose


def remove_empty_lines(file_path : str):
    # Read the contents of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Keep only non-empty lines
    non_empty_lines = [line for line in lines if line.strip() != '']

    # Write the cleaned lines back to the file
    with open(file_path, 'w') as file:
        file.writelines(non_empty_lines)


def format_world_file(workspace_path : str,
                         package_name : str,
                         env_model : list[str],
                         x_offset : int, 
                         y_offset : int, 
                         namespace : str) -> None:
    try:
        template_file_path = os.path.join(workspace_path, 'src', package_name, 'worlds', f'{env_model}.world')
        generated_file_path = os.path.join(workspace_path, 'src', package_name, 'extras', 'worlds_models_text', f'{env_model}.xml')

        # Parse the XML file
        tree = ET.parse(template_file_path)
        root = tree.getroot()

        with open(generated_file_path, 'w', encoding='utf-8') as file:
            # Find all <model> elements and add them to the new root
            for model in root.findall(".//model"):
                name = model.get('name')
                pose = model.find('pose')

                if pose is not None:
                    # Remove existing 'env_' followed by numbers from the name
                    name = re.sub(r'env_\d+_', '', name)
                    # Update the name with the new namespace
                    model.set('name', f"{namespace}{name}")

                    x, y, z, roll, pitch, yaw = map(float, pose.text.split())
                    pose.text = f"{x + x_offset} {y + y_offset} {z} {roll} {pitch} {yaw}"
                else: #try to find the <pose> tag inside the <include> tag of the <model> tag
                    include = model.find('include')
                    pose = include.find('pose')
                    if pose is not None:

                        # Remove existing 'env_' followed by numbers from the name
                        name = re.sub(r'env_\d+_', '', name)
                        # Update the name with the new namespace
                        model.set('name', f"{namespace}{name}")

                        x, y, z, roll, pitch, yaw = map(float, pose.text.split())
                        pose.text = f"{x + x_offset} {y + y_offset} {z} {roll} {pitch} {yaw}"

                    else:
                        raise Exception(f"Pose not found for model '{name}' inside {generated_file_path}.")
                file.write(ET.tostring(model, encoding='unicode'))




    except Exception as e:
        print(f"[ERROR] [utils]: {e}")


def create_rviz_config(rviz_config_dir : str, namespace : str) -> str:
    try:

        with open(os.path.join(rviz_config_dir, 'rviz_config_template.txt'), 'r') as file:
            template_lines = file.readlines()

        # Flags to track when inside a Topic or Update Topic block
        inside_topic_block = False
        config_lines = []

        for line in template_lines:
            stripped_line = line.strip()
            
            # Detect the start of a Topic or Update Topic block
            if stripped_line.startswith("Topic:") or stripped_line.startswith("Update Topic:"):
                inside_topic_block = True
                # don't modify next line!!! the identation is important!!
                topic_indentation = f"  {line[:len(line) - len(line.lstrip())]}"

            # Check for "Value:" lines within a Topic or Update Topic block
            if inside_topic_block and stripped_line.startswith("Value:"):
                # Split the line to get the topic name, add the namespace, and reformat the line
                topic_name = stripped_line.split("Value:", 1)[1].strip()
                line = f"{topic_indentation}Value: /{namespace}{topic_name}\n"
                inside_topic_block = False

            # Append the modified or unmodified line to the new lines list
            config_lines.append(line)
        

        config_path = os.path.join(rviz_config_dir, f'{namespace}_config.rviz')
        with open(config_path, 'w') as file:
            file.writelines(config_lines)
            
        return config_path
        
    except Exception as e:
        print(f"[ERROR] [utils]: {e}")
        return None