# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('isaac_orbbec_launch'),
        'config', 'sensors', 'orbbec.yaml')
    orbbec_camera_launch_file = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch', 'gemini_330_series.launch.py')

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='orbbec_container')

    # If we do not attach to a shared component container we have to create our own container.
    orbbec_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # RealSense splitter node
            # ComposableNode(
            #     namespace="camera",
            #     name='orbbec_camera_splitter_node',
            #     package='orbbec_camera_splitter',
            #     plugin='nvblox::OrbbecCameraSplitterNode',
            #     parameters=[{
            #         'input_qos': 'DEFAULT',
            #         'output_qos': 'SENSOR_DATA',
            #     }],
            #     remappings=[('input/infra_1', '/camera/left_ir/image_raw'),
            #                 ('input/infra_1_metadata', '/camera/left_ir/metadata'),
            #                 ('input/infra_2', '/camera/right_ir/image_raw'),
            #                 ('input/infra_2_metadata', '/camera/right_ir/metadata'),
            #                 ('input/depth', '/camera/depth/image_raw'),
            #                 ('input/depth_metadata', '/camera/depth/metadata'),
            #                 ('input/pointcloud', '/camera/depth_registered/points'),
            #                 ('input/pointcloud_metadata',
            #                  '/camera/depth/metadata'),
            #                 ]),
            ComposableNode(
              namespace="camera",
              # name='orbbec_camera_splitter_node',
              name='orbbec_camera_node',
              package='orbbec_camera',
              plugin='orbbec_camera::OBCameraNodeDriver',
              parameters=[
                  config_file
                ],
              remappings=[
                          ('/camera/left_ir/image_raw', '~/output/infra_1'),
                          ('/camera/right_ir/image_raw', '~/output/infra_2'),
                          ('/camera/depth/image_raw', '~/output/depth'),
                          ('/camera/depth_registered/points', '~/output/pointcloud'),
                      ]),
            ]
        )

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orbbec_camera_launch_file]),
        launch_arguments={'config_file_path': config_file}.items())


    # return LaunchDescription([orbbec_container, load_composable_nodes, orbbec_launch])
    return LaunchDescription([orbbec_container, load_composable_nodes])
