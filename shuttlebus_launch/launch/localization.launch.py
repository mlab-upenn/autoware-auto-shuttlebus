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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
"""Launch file for vehicle for Autoware bootcamp (Lincoln MKZ @Pennovation)."""

import os
from ament_index_python import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_shared_file_path(package_name, folder_name, file_name):
    """Get the full path of the shared file."""
    return os.path.join(get_package_share_directory(package_name), folder_name,
                        file_name)


def get_shared_file(package_name, folder_name, file_name, arg_name):
    """Pass the given shared file as a LaunchConfiguration."""
    file_path = os.path.join(get_package_share_directory(package_name),
                             folder_name, file_name)
    return launch.substitutions.LaunchConfiguration(arg_name,
                                                    default=[file_path])


def generate_launch_description():
    """Generate launch description with a single component."""
    with open(
            get_shared_file_path('bootcamp_launch', 'urdf',
                                 'lincoln_mkz_17.urdf'), 'r') as infp:
        urdf_file = infp.read()

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file
        }],
    )

    # -------------------------------------------------------
    # Launch Ouster Lidar
    ouster_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        get_package_share_directory('ros2_ouster'), '/launch/driver_launch.py'
    ]),
                                             launch_arguments={
                                                 'params_file':
                                                 get_shared_file_path(
                                                     'bootcamp_launch',
                                                     'config',
                                                     'ouster_config.yaml')
                                             }.items())

    # Launch Point type converter node
    point_type_adapter = Node(package='point_type_adapter',
                              executable='point_type_adapter_node_exe',
                              name='point_type_adapter_node',
                              namespace='ouster',
                              output='screen',
                              remappings=[("points_raw", "/points")])
    # Launch Point cloud filter transformer
    filter_transformer_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/ouster_filter_transformer.param.yaml')
    filter_transformer_param = DeclareLaunchArgument(
        'filter_transformer_param_file',
        default_value=filter_transformer_param_file,
        description='Path to parameter file for Point Cloud Filter Transformer'
    )
    filter_transformer = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='point_cloud_filter_transform_node',
        namespace='ouster',
        output='screen',
        parameters=[
            launch.substitutions.LaunchConfiguration(
                'filter_transformer_param_file')
        ],
        remappings=[('points_in', 'points_xyzi')])

    # ----------------- Mapping ---------------------
    # Launch Lanlet2 map
    map_osm_file = os.path.join(get_package_share_directory('bootcamp_launch'),
                                'data/Pennovation.osm')
    lanelet2_map_provider_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/lanelet2_map_provider.param.yaml')
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider')
    lanelet2_map_provider = Node(package='lanelet2_map_provider',
                                 executable='lanelet2_map_provider_exe',
                                 namespace='had_maps',
                                 name='lanelet2_map_provider_node',
                                 parameters=[
                                     launch.substitutions.LaunchConfiguration(
                                         'lanelet2_map_provider_param_file'), {
                                             'map_osm_file': map_osm_file
                                         }
                                 ])
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps',
        parameters=[
            launch.substitutions.LaunchConfiguration(
                'lanelet2_map_provider_param_file'), {
                    'map_osm_file': map_osm_file
                }
        ])

    # Launch point cloud map
    map_pcd_file = os.path.join(get_package_share_directory('bootcamp_launch'),
                                'data/Pennovation_processed.pcd')
    map_yaml_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'data/Pennovation.yaml')
    map_publisher_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/map_publisher.param.yaml')
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher')
    map_publisher = Node(package='ndt_nodes',
                         executable='ndt_map_publisher_exe',
                         namespace='localization',
                         parameters=[
                             launch.substitutions.LaunchConfiguration(
                                 'map_publisher_param_file'), {
                                     'map_pcd_file': map_pcd_file,
                                     'map_yaml_file': map_yaml_file
                                 }
                         ])
    # --------------------------------------------------

    # ----------------- NDT Localizer -----------------
    scan_downsampler_param_file_path = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/scan_downsampler.param.yaml')
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file_path',
        default_value=scan_downsampler_param_file_path,
        description='Path to config file for scan downsampler')
    scan_downsampler = Node(package='voxel_grid_nodes',
                            executable='voxel_grid_node_exe',
                            namespace='lidars',
                            name='voxel_grid_cloud_node',
                            parameters=[
                                launch.substitutions.LaunchConfiguration(
                                    'scan_downsampler_param_file_path')
                            ],
                            remappings=[("points_in",
                                         "/ouster/points_filtered")])
    ndt_localizer_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/ndt_localizer.param.yaml')
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer')
    ndt_localizer = Node(package='ndt_nodes',
                         executable='p2d_ndt_localizer_exe',
                         namespace='localization',
                         name='p2d_ndt_localizer_node',
                         parameters=[
                             launch.substitutions.LaunchConfiguration(
                                 'ndt_localizer_param_file')
                         ],
                         remappings=[("points_in",
                                      "/lidars/points_downsampled"),
                                     ("observation_republish",
                                      "viz_points_downsampled")])
    # -----------------------------------------------------

    # --------------- Launch RViz2 ------------------------
    rviz_cfg_path = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'rviz/localization.rviz')
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            launch.substitutions.LaunchConfiguration("rviz_cfg_path_param")
        ],
        condition=IfCondition(
            launch.substitutions.LaunchConfiguration('with_rviz')),
        remappings=[("initialpose", "/localization/initialpose"),
                    ("goal_pose", "/planning/goal_pose")],
    )

    return LaunchDescription([
        # ouster_launch,
        point_type_adapter,
        urdf_publisher,
        filter_transformer_param,
        filter_transformer,
        map_publisher_param,
        map_publisher,
        lanelet2_map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        with_rviz_param,
        rviz_cfg_path_param,
        rviz2,
        scan_downsampler_param,
        scan_downsampler,
        ndt_localizer_param,
        ndt_localizer,
    ])
