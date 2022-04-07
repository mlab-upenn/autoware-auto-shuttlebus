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
    dataspeed_ford_dbw = Node(executable='dbw_node',
                              name='dataspeed_ford_dbw_node',
                              namespace='vehicle',
                              package='dbw_ford_can',
                              parameters=[
                                  get_shared_file('bootcamp_launch', 'config',
                                                  'dbw_params.yaml', 'params')
                              ],
                              output='screen')

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

    tf = Node(package='tf2_ros',
              executable='static_transform_publisher',
              arguments=[
                  '0', '0', '0', '0', '0', '0', '1', 'base_link',
                  'laser_data_frame'
              ])

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(get_package_share_directory('bootcamp_launch'),
                             'config', 'lidarslam.yaml'))
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

    # ----------------- lLidar SLAM ROS2 Mapping -----------------
    mapping = Node(package='scanmatcher',
                   executable='scanmatcher_node',
                   parameters=[main_param_dir],
                   remappings=[('/input_cloud', '/points')],
                   output='screen')

    graphbasedslam = Node(package='graph_based_slam',
                          executable='graph_based_slam_node',
                          parameters=[main_param_dir],
                          output='screen')
    # -----------------------------------------------------

    # ----------------- NDT Mapping(DEPRECATED) -----------------
    # # Launch NDT Mapping node
    # ndt_mapper_param_file = os.path.join(
    #     get_package_share_directory('bootcamp_launch'),
    #     'config/ndt_mapper.param.yaml')
    # ndt_mapper_param = DeclareLaunchArgument(
    #     'ndt_param_param_file',
    #     default_value=ndt_mapper_param_file,
    #     description='Path to config file for ndt mapper')
    # ndt_mapper = Node(
    #     package='ndt_mapping_nodes',
    #     executable='ndt_mapper_node_exe',
    #     name='ndt_mapper_node',
    #     namespace='mapper',
    #     output='screen',
    #     parameters=[
    #         launch.substitutions.LaunchConfiguration('ndt_param_param_file')
    #     ],
    #     remappings=[("points_in", "/points_xyzi"),
    #                 ("points_registered", "/points_registered")])
    # -----------------------------------------------------

    # Launch Point type converter node
    point_type_adapter = Node(package='point_type_adapter',
                              executable='point_type_adapter_node_exe',
                              name='point_type_adapter_node',
                              namespace='',
                              output='screen',
                              remappings=[("/points_raw", "/points")])

    # ----------------- Mapping ---------------------
    # Launch Lanlet2 map
    # TODO(Zhihao): complete the launch configuration file: param/lanelet2_map_provider.param.yaml
    lanelet2_map_provider_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'), 'param/lanelet2_map_provider.param.yaml')
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[launch.substitutions.LaunchConfiguration('lanelet2_map_provider_param_file')]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )

    # Launch point cloud map
    map_publisher_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'), 'param/map_publisher.param.yaml')
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[launch.substitutions.LaunchConfiguration('map_publisher_param_file')]
    )
    # --------------------------------------------------

    # ----------------- NDT Localizer -----------------
    ndt_localizer_param_file = os.path.join(
        get_package_share_directory('bootcamp_launch'),
        'config/ndt_localizer.param.yaml')
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[launch.substitutions.LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled"),
            ("observation_republish", "/lidars/points_fused_viz"),
        ]
    )
    # -----------------------------------------------------

    return LaunchDescription([
        # dataspeed_ford_dbw,
        urdf_publisher,
        tf,
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        map_publisher_param,
        map_publisher,
        # ouster_launch,
        # point_type_adapter,
        # mapping,
        # graphbasedslam,
        # ndt_localizer_param,
        # ndt_localizer,
    ])
