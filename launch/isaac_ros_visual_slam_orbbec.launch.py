# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    orbbec_camera_node = Node(
        name='ob_camera_node',
        package="orbbec_camera",
        executable="orbbec_camera_node",
        namespace='camera',
        parameters=[{
                'enable_ir_auto_exposure' : False,
                'ir_exposure' : 5000,
                'ir_gain' : 24,
                'enable_laser': False,
                'enable_left_ir': True,
                'left_ir_width': 640,
                'left_ir_height': 480,
                'left_ir_fps' : 60,
                'left_ir_format': 'Y8',
                'enable_right_ir': True,
                'right_ir_width': 640,
                'right_ir_height': 480,
                'right_ir_fps' : 60,
                'right_ir_format': 'Y8',
                'enable_color': False,
                'enable_depth': False,
                'enable_sync_output_accel_gyro': True,
                'accel_rate': '200hz',
                'accel_range': '4g',
                'gyro_rate': '200hz',
                'gyro_range': '1000dps',
        }]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': False,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 33.00
                    }],
        remappings=[('stereo_camera/left/image', '/camera/left_ir/image_raw'),
                    ('stereo_camera/left/camera_info', '/camera/left_ir/camera_info'),
                    ('stereo_camera/right/image', '/camera/right_ir/image_raw'),
                    ('stereo_camera/right/camera_info', '/camera/right_ir/camera_info'),
                    ('visual_slam/imu', '/camera/gyro_accel/sample')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container, orbbec_camera_node])
