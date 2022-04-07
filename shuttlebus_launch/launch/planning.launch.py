# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
  """
  Launch planning nodes.

    * behavior_planner
    * lanelet2_global_planner
    * lane_planner
    * mpc_controller
    * object_collision_estimator
    * costmap_generator
    * freespace_planner
  """
  bootcamp_launch_pkg_prefix = get_package_share_directory('bootcamp_launch')
  behavior_planner_param_file = os.path.join(
      bootcamp_launch_pkg_prefix, 'config/behavior_planner.param.yaml')
  lane_planner_param_file = os.path.join(bootcamp_launch_pkg_prefix,
                                         'config/lane_planner.param.yaml')
  mpc_param_file = os.path.join(bootcamp_launch_pkg_prefix,
                                'config/mpc_controller.param.yaml')
  object_collision_estimator_param_file = os.path.join(
      bootcamp_launch_pkg_prefix,
      'config/object_collision_estimator.param.yaml')
  costmap_generator_param_file = os.path.join(
      bootcamp_launch_pkg_prefix, 'config/costmap_generator.param.yaml')
  freespace_planner_param_file = os.path.join(
      bootcamp_launch_pkg_prefix, 'config/freespace_planner.param.yaml')
  vehicle_characteristics_param_file = os.path.join(
      bootcamp_launch_pkg_prefix, 'config/vehicle_characteristics.param.yaml')
  vehicle_constants_manager_param_file = os.path.join(
      bootcamp_launch_pkg_prefix, 'config/lexus_rx_hybrid_2016.param.yaml')

  # Arguments
  with_obstacles_param = DeclareLaunchArgument(
      'with_obstacles',
      default_value='False',
      description='Enable obstacle detection')
  behavior_planner_param = DeclareLaunchArgument(
      'behavior_planner_param_file',
      default_value=behavior_planner_param_file,
      description='Path to parameter file for behavior planner')
  lane_planner_param = DeclareLaunchArgument(
      'lane_planner_param_file',
      default_value=lane_planner_param_file,
      description='Path to parameter file for lane planner')
  mpc_param = DeclareLaunchArgument(
      'mpc_param_file',
      default_value=mpc_param_file,
      description='Path to config file for MPC')
  object_collision_estimator_param = DeclareLaunchArgument(
      'object_collision_estimator_param_file',
      default_value=object_collision_estimator_param_file,
      description='Path to parameter file for object collision estimator')
  costmap_generator_param = DeclareLaunchArgument(
      'costmap_generator_param_file',
      default_value=costmap_generator_param_file,
      description='Path to parameter file for costmap generator')
  freespace_planner_param = DeclareLaunchArgument(
      'freespace_planner_param_file',
      default_value=freespace_planner_param_file,
      description='Path to parameter file for freespace planner')
  vehicle_characteristics_param = DeclareLaunchArgument(
      'vehicle_characteristics_param_file',
      default_value=vehicle_characteristics_param_file,
      description='Path to config file for vehicle characteristics')
  vehicle_constants_manager_param = DeclareLaunchArgument(
      'vehicle_constants_manager_param_file',
      default_value=vehicle_constants_manager_param_file,
      description='Path to config file for vehicle_constants_manager')

  # Nodes
  behavior_planner = Node(
      package='behavior_planner_nodes',
      name='behavior_planner_node',
      namespace='planning',
      executable='behavior_planner_node_exe',
      parameters=[
          LaunchConfiguration('behavior_planner_param_file'),
          {
              'enable_object_collision_estimator':
                  LaunchConfiguration('with_obstacles')
          },
          LaunchConfiguration('vehicle_characteristics_param_file'),
      ],
      output='screen',
      remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
                  ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
                  ('route', 'global_path'),
                  ('vehicle_state_report', '/vehicle/state_report'),
                  ('vehicle_state_command', '/vehicle/state_command')])
  lanelet2_global_planner = Node(
      package='lanelet2_global_planner_nodes',
      name='lanelet2_global_planner_node',
      namespace='planning',
      executable='lanelet2_global_planner_node_exe',
      remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                  ('vehicle_kinematic_state',
                   '/vehicle/vehicle_kinematic_state')])
  lane_planner = Node(
      package='lane_planner_nodes',
      name='lane_planner_node',
      namespace='planning',
      executable='lane_planner_node_exe',
      parameters=[
          LaunchConfiguration('lane_planner_param_file'),
          LaunchConfiguration('vehicle_characteristics_param_file'),
      ],
      remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')])
  mpc_controller = Node(
      package='mpc_controller_nodes',
      executable='mpc_controller_node_exe',
      name='mpc_controller_node',
      namespace='control',
      parameters=[
          LaunchConfiguration('mpc_param_file'),
          LaunchConfiguration('vehicle_characteristics_param_file'),
      ],
  )
  object_collision_estimator = Node(
      package='object_collision_estimator_nodes',
      name='object_collision_estimator_node',
      namespace='planning',
      executable='object_collision_estimator_node_exe',
      condition=IfCondition(LaunchConfiguration('with_obstacles')),
      parameters=[
          LaunchConfiguration('object_collision_estimator_param_file'),
          LaunchConfiguration('vehicle_characteristics_param_file'),
      ],
      remappings=[
          ('obstacle_topic', '/perception/lidar_bounding_boxes_filtered'),
      ])
  costmap_generator = Node(
      package='costmap_generator_nodes',
      executable='costmap_generator_node_exe',
      name='costmap_generator_node',
      namespace='planning',
      output='screen',
      parameters=[
          LaunchConfiguration('costmap_generator_param_file'),
      ],
      remappings=[('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')])
  freespace_planner = Node(
      package='freespace_planner_nodes',
      executable='freespace_planner_node_exe',
      name='freespace_planner',
      namespace='planning',
      output='screen',
      parameters=[
          LaunchConfiguration('freespace_planner_param_file'),
          LaunchConfiguration('vehicle_constants_manager_param_file')
      ])

  return LaunchDescription([
      with_obstacles_param, vehicle_constants_manager_param,
      behavior_planner_param, lane_planner_param, mpc_param,
      object_collision_estimator_param, costmap_generator_param,
      freespace_planner_param, vehicle_characteristics_param, behavior_planner,
      lanelet2_global_planner, lane_planner, mpc_controller,
      object_collision_estimator, costmap_generator, freespace_planner
  ])
