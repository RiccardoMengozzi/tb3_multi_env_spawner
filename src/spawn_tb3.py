#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Script used to spawn a robot in a generic position."""

import argparse
import xml.etree.ElementTree as ET

from gazebo_msgs.srv import SpawnEntity
import rclpy


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-urdf', '--robot_urdf', type=str, default='dummy.urdf',
                        help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-yaw', type=float, default=0,
                        help='the yaw angle of the initial position [meters]')

    args, unknown = parser.parse_known_args()


    # Start node
    rclpy.init()
    node = rclpy.create_node('tb3_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    urdf_file_path = args.robot_urdf
    print(urdf_file_path)

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the urdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf

    tree = ET.parse(urdf_file_path)
    root = tree.getroot()
    imu_plugin = None
    diff_drive_plugin = None 
    for plugin in root.iter('plugin'):
        if 'turtlebot3_diff_drive' in plugin.attrib.values():
            diff_drive_plugin = plugin
        elif 'turtlebot3_imu' in plugin.attrib.values():
            imu_plugin = plugin
        elif 'turtlebot3_joint_state' in plugin.attrib.values():
            joint_state_plugin = plugin

    # We change the namespace to the robots corresponding one
    if diff_drive_plugin is not None:
        tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
        tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
        tag_diff_drive_ns.text = '/' + args.robot_namespace
        tag_diff_drive_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
        tag_diff_drive_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
        tag_diff_drive_tf_static_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
        tag_diff_drive_tf_static_remap.text = '/tf_static:=/' + args.robot_namespace + '/tf_static'
    else:
        print("ERROR>>>>>>>>>>>>>>>>>>>>> DIFF DRIVE NOT FOUND")

    if joint_state_plugin is not None:
        tag_joint_state_ros_params = joint_state_plugin.find('ros')
        tag_joint_state_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
        tag_joint_state_ns.text = '/' + args.robot_namespace
        tag_joint_state_tf_remap = ET.SubElement(tag_joint_state_ros_params, 'remapping')
        tag_joint_state_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
        tag_joint_state_tf_static_remap = ET.SubElement(tag_joint_state_ros_params, 'remapping')
        tag_joint_state_tf_static_remap.text = '/tf_static:=/' + args.robot_namespace + '/tf_static'
    else:
        print("ERROR>>>>>>>>>>>>>>>>>>>>> JOINT STATE NOT FOUND")

    if imu_plugin is not None:
        tag_imu_ros_params = imu_plugin.find('ros')
        tag_imu_ns = ET.SubElement(tag_imu_ros_params, 'namespace')
        tag_imu_ns.text = '/' + args.robot_namespace + '/imu'
        tag_imu_tf_remap = ET.SubElement(tag_imu_ros_params, 'remapping')
        tag_imu_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
        tag_imu_tf_static_remap = ET.SubElement(tag_imu_ros_params, 'remapping')
        tag_imu_tf_static_remap.text = '/tf_static:=/' + args.robot_namespace + '/tf_static'
    else:
        print("ERROR>>>>>>>>>>>>>>>>>>>>> IMU NOT FOUND")


    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)
    request.initial_pose.orientation.z = float(args.yaw)


    if args.namespace is True:
        node.get_logger().info(f'spawn `{args.robot_name}` at {args.x}, {args.y}, {args.z}, with yaw {args.yaw}, with namespace `{args.robot_namespace}`')

        request.robot_namespace = args.robot_namespace

    else:
        node.get_logger().info(f'spawn `{args.robot_name}` at {args.x}, {args.y}, {args.z}, with yaw {args.yaw}')

    node.get_logger().info('Spawning Robot using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
