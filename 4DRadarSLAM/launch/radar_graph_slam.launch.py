from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('radar_graph_slam')
    config = os.path.join(pkg, 'config', 'params_ros2.yaml')

    # ── arguments (mirrors the original <arg> tags) ──────────────────────────
    enable_barometer          = LaunchConfiguration('enable_barometer',          default='false')
    enable_gps                = LaunchConfiguration('enable_gps',                default='true')
    enable_dynamic_object_removal = LaunchConfiguration('enable_dynamic_object_removal', default='false')
    # enable_frontend_ego_vel   = LaunchConfiguration('enable_frontend_ego_vel',   default='true') # was false
    enable_preintegration     = LaunchConfiguration('enable_preintegration', default='false')
    enable_transform_thresholding = LaunchConfiguration('enable_transform_thresholding', default='false')
    enable_loop_closure       = LaunchConfiguration('enable_loop_closure',       default='false')
    auto_export_on_bag_exit   = LaunchConfiguration('auto_export_on_bag_exit',   default='true')
    keyframe_delta_trans_front_end = LaunchConfiguration('keyframe_delta_trans_front_end', default='0.25')
    keyframe_delta_trans_back_end  = LaunchConfiguration('keyframe_delta_trans_back_end',  default='2.0')
    keyframe_delta_angle      = LaunchConfiguration('keyframe_delta_angle',      default='0.2612')
    registration_method       = LaunchConfiguration('registration_method',       default='GICP')
    reg_resolution            = LaunchConfiguration('reg_resolution',            default='1.0')
    dist_var                  = LaunchConfiguration('dist_var',                  default='0.86')
    azimuth_var               = LaunchConfiguration('azimuth_var',               default='0.5')
    elevation_var             = LaunchConfiguration('elevation_var',             default='1.0')
    bag_path                  = LaunchConfiguration('bag_path',
        default='/home/tianyu-tony-zhou/dataset/rosbag_2025_07_17-15-11-39_lidar_imu_bosch_comp-zed')

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '--clock',
            #'--topics',
            #'/off_highway_premium_radar_sample_driver/locations',
            #'/imu/data',
            #'/gnss',
            #'/tf',
            #'/tf_static',
            #'/zed2i/zed_node/left/image_rect_color/compressed',
            #'/zed2i/zed_node/left/camera_info',
        ],
        output='screen',
    )

    auto_export = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'topic', 'pub', '--once',
                                '/command',
                                'std_msgs/msg/String',
                                '{data: output_aftmapped}',
                            ],
                            output='screen',
                            condition=IfCondition(auto_export_on_bag_exit),
                        ),
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([

        # ── preprocessing ─────────────────────────────────────────────────────
        Node(
            package='radar_graph_slam',
            executable='preprocessing_node',
            name='preprocessing',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'use_distance_filter': True,
                'use_azimuth_filter': True,
                'scan_azimuth_min_deg': -85.0,
                'scan_azimuth_max_deg': 85.0,
                'distance_near_thresh': 1.5,
                'distance_far_thresh': 120.0,
                'z_low_thresh': -5.0,
                'z_high_thresh': 100.0,
                'downsample_method': 'VOXELGRID',
                'downsample_resolution': 0.1,
                'outlier_removal_method': 'NONE',
                'statistical_mean_k': 30,
                'statistical_stddev': 1.2,
                'radius_radius': 0.7,
                'radius_min_neighbors': 2,
                'power_threshold': 1.0,
                'enable_dynamic_object_removal': enable_dynamic_object_removal,
            }]
        ),

        # ── scan matching odometry ────────────────────────────────────────────
        Node(
            package='radar_graph_slam',
            executable='scan_matching_odometry_node',
            name='scan_matching_odometry',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'keyframe_delta_trans': keyframe_delta_trans_front_end,
                'keyframe_delta_angle': keyframe_delta_angle,
                'keyframe_min_size': 100,
                'enable_transform_thresholding': True,
                'enable_imu_thresholding': False,
                'max_acceptable_trans': 1.5,
                'max_acceptable_angle': 1.0,
                'max_diff_trans': 1.0,
                'max_diff_angle': 1.0,
                'max_egovel_cum': 2.0,
                'enable_planar_motion': True,
                'downsample_method': 'NONE',
                'downsample_resolution': 0.1,
                'registration_method': registration_method,
                'dist_var': dist_var,
                'azimuth_var': azimuth_var,
                'elevation_var': elevation_var,
                'reg_num_threads': 0,
                'reg_transformation_epsilon': 0.1,
                'reg_maximum_iterations': 64,
                'reg_max_correspondence_distance': 2.0,
                'reg_max_optimizer_iterations': 20,
                'reg_use_reciprocal_correspondences': False,
                'reg_correspondence_randomness': 8,
                'reg_resolution': reg_resolution,
                'reg_nn_search_method': 'DIRECT7',
                'min_registration_points': 25,
                'use_ego_vel': True,
                'max_submap_frames': 5,
                'enable_scan_to_map': False,
                'enable_imu_fusion': False,
                'imu_debug_out': False,
                'imu_fusion_ratio': 0.05,
            }]
        ),

        # ── radar graph slam ──────────────────────────────────────────────────
        Node(
            package='radar_graph_slam',
            executable='radar_graph_slam_node',
            name='radar_graph_slam',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'g2o_solver_type': 'lm_var_cholmod',
                'g2o_solver_num_iterations': 512,
                'enable_barometer': enable_barometer,
                'enable_gps': False,
                'max_keyframes_per_update': 30,
                'keyframe_delta_trans': keyframe_delta_trans_back_end,
                'keyframe_delta_angle': keyframe_delta_angle,
                'keyframe_min_size': 500,
                'fix_first_node': True,
                'fix_first_node_stddev': '10 10 10 1 1 1',
                'fix_first_node_adaptive': True,
                'enable_loop_closure': False,
                'enable_pf': True,
                'enable_odom_check': True,
                'distance_thresh': 10.0,
                'accum_distance_thresh': 50.0,
                'min_loop_interval_dist': 10.0,
                'max_baro_difference': 2.0,
                'max_yaw_difference': 20.0,
                'sc_dist_thresh': 0.5,
                'sc_azimuth_range': 56.5,
                'historyKeyframeFitnessScore': 6.0,
                'odom_check_trans_thresh': 0.3,
                'odom_check_rot_thresh': 0.05,
                'pairwise_check_trans_thresh': 1.5,
                'pairwise_check_rot_thresh': 0.2,
                'registration_method': registration_method,
                'reg_num_threads': 0,
                'reg_transformation_epsilon': 0.1,
                'reg_maximum_iterations': 64,
                'reg_max_correspondence_distance': 2.0,
                'reg_max_optimizer_iterations': 20,
                'reg_use_reciprocal_correspondences': False,
                'reg_correspondence_randomness': 8,
                'reg_resolution': reg_resolution,
                'reg_nn_search_method': 'DIRECT7',
                'barometer_edge_type': 1,
                'barometer_edge_robust_kernel': 'Huber',
                'barometer_edge_robust_kernel_size': 1.0,
                'barometer_edge_stddev': 0.47,
                'gps_edge_robust_kernel': 'Huber',
                'gps_edge_robust_kernel_size': 1.0,
                'gps_edge_stddev_xy': 5.0,
                'gps_edge_stddev_z': 5.0,
                'max_gps_edge_stddev_xy': 1.5,
                'max_gps_edge_stddev_z': 3.0,
                'gps_edge_intervals': 15,
                'dataset_name': 'loop2',
                'enable_preintegration': enable_preintegration,
                'enable_planar_z_constraint': True,
                'planar_z_constraint_stddev': 0.05,
                'enable_gravity_constraint': True,
                'gravity_constraint_stddev': 0.05,
                'enable_imu_orientation': True,
                'imu_orientation_stddev': 0.2,
                'use_egovel_preinteg_trans': False,
                'preinteg_orient_stddev': 1.0,
                'preinteg_trans_stddev': 5.0,
                'odometry_edge_robust_kernel': 'Huber',
                'odometry_edge_robust_kernel_size': 1.0,
                'loop_closure_edge_robust_kernel': 'Huber',
                'loop_closure_edge_robust_kernel_size': 1.0,
                'graph_update_interval': 2.0,
                'map_cloud_update_interval': 6.0,
                'map_cloud_resolution': 0.05,
                'show_sphere': False,
            }]
        ),

        # ── RViz2 ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_slam',
            arguments=['-d', os.path.join(pkg, 'rviz', 'radar_graph_slam.rviz')],
            output='screen',
        ),

        # ── bag playback (replaces rosbag_play_radar_carpark1.launch) ─────────
        bag_play,
        auto_export,
    ])
