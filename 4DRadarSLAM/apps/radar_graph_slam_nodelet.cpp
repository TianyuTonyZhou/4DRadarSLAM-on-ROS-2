// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/octree/octree_search.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <radar_graph_slam/srv/save_map.hpp>
#include <radar_graph_slam/srv/dump_graph.hpp>
#include <radar_graph_slam/ros_utils.hpp>
#include <radar_graph_slam/ros_time_hash.hpp>
#include <radar_graph_slam/msg/floor_coeffs.hpp>
#include <radar_graph_slam/graph_slam.hpp>
#include <radar_graph_slam/keyframe.hpp>
#include <radar_graph_slam/keyframe_updater.hpp>
#include <radar_graph_slam/loop_detector.hpp>
#include <radar_graph_slam/information_matrix_calculator.hpp>
#include <radar_graph_slam/map_cloud_generator.hpp>
#include <radar_graph_slam/nmea_sentence_parser.hpp>
#include "radar_graph_slam/polynomial_interpolation.hpp"
#include <radar_graph_slam/registrations.hpp>

#include "scan_context/Scancontext.h"

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

#include <barometer_bmp388/msg/barometer.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "utility_radar.h"

using namespace std;

namespace radar_graph_slam {

class RadarGraphSlamComponent : public rclcpp::Node, public ParamServer {
public:
  typedef pcl::PointXYZI PointT;
  typedef PointXYZIRPYT  PointTypePose;

  using OdomMsg  = nav_msgs::msg::Odometry;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<OdomMsg, CloudMsg>;

  explicit RadarGraphSlamComponent(const rclcpp::NodeOptions & options)
  : Node("radar_graph_slam", options) , ParamServer(this)
  {

    // init parameters
    map_cloud_resolution = declare_parameter<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();
    trans_aftmapped.setIdentity();
    trans_aftmapped_incremental.setIdentity();
    initial_pose.setIdentity();

    max_keyframes_per_update = declare_parameter<int>("max_keyframes_per_update", 10);

    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(declare_parameter<std::string>("g2o_solver_type", "lm_var_cholmod")));
    map_cloud_generator.reset(new MapCloudGenerator());
    nmea_parser.reset(new NmeaSentenceParser());

    gps_edge_intervals = declare_parameter<int>("gps_edge_intervals", 10);
    gps_time_offset = declare_parameter<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = declare_parameter<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = declare_parameter<double>("gps_edge_stddev_z", 10.0);
    max_gps_edge_stddev_xy = declare_parameter<double>("max_gps_edge_stddev_xy", 1.0);
    max_gps_edge_stddev_z = declare_parameter<double>("max_gps_edge_stddev_z", 2.0);

    // Preintegration Parameters
    enable_preintegration = declare_parameter<bool>("enable_preintegration", false);
    use_egovel_preinteg_trans = declare_parameter<bool>("use_egovel_preinteg_trans", false);
    preinteg_trans_stddev = declare_parameter<double>("preinteg_trans_stddev", 1.0);
    preinteg_orient_stddev = declare_parameter<double>("preinteg_orient_stddev", 2.0);

    enable_barometer = declare_parameter<bool>("enable_barometer", false);
    barometer_edge_type = declare_parameter<int>("barometer_edge_type", 2);
    barometer_edge_stddev = declare_parameter<double>("barometer_edge_stddev", 0.5);

    if (!has_parameter("points_topic"))
        declare_parameter<std::string>("points_topic", "/radar_enhanced_pcl");
    points_topic = get_parameter("points_topic").get_value<std::string>();

    show_sphere = declare_parameter<bool>("show_sphere", false);

    dataset_name = declare_parameter<std::string>("dataset_name", "");

    // subscribers
    odom_sub = std::make_shared<message_filters::Subscriber<OdomMsg>>(this, odomTopic);
    cloud_sub = std::make_shared<message_filters::Subscriber<CloudMsg>>(this, "/filtered_points");
    sync = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub);
    sync->registerCallback(std::bind(&RadarGraphSlamComponent::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    if(declare_parameter<bool>("enable_gps", true)) {
      gps_sub = create_subscription<geographic_msgs::msg::GeoPointStamped>(
        "/gps/geopoint", 
        1024, 
        std::bind(&RadarGraphSlamComponent::gps_callback, 
          this, 
          std::placeholders::_1)
        );
      nmea_sub = create_subscription<nmea_msgs::msg::Sentence>(
        "/gpsimu_driver/nmea_sentence", 
        1024, 
        std::bind(&RadarGraphSlamComponent::nmea_callback, 
          this, 
          std::placeholders::_1)
        );
      navsat_sub = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, 
        1024, 
        std::bind(&RadarGraphSlamComponent::navsat_callback, 
          this, 
          std::placeholders::_1)
        );
    }
    if(enable_barometer) {
      barometer_sub = create_subscription<barometer_bmp388::msg::Barometer>("/barometer/filtered", 
        16, 
        std::bind(&RadarGraphSlamComponent::barometer_callback, 
          this, 
          std::placeholders::_1)
        );
    }
    if (enable_preintegration)
      imu_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
    "/imu_pre_integ/imu_odom_incre", 
    1024, 
    std::bind(&RadarGraphSlamComponent::imu_odom_callback, 
      this, 
      std::placeholders::_1));
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 
      1024, 
      std::bind(&RadarGraphSlamComponent::imu_callback, 
        this, 
        std::placeholders::_1)
      );
    command_sub = create_subscription<std_msgs::msg::String>(
      "/command", 
      10, 
      std::bind(&RadarGraphSlamComponent::command_callback, 
        this, 
        std::placeholders::_1)
      );

    //***** publishers ******
    markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/radar_graph_slam/markers", 16);
    // Transform RadarOdom_to_base
    odom2base_pub = create_publisher<geometry_msgs::msg::TransformStamped>("/radar_graph_slam/odom2base", 16);
    aftmapped_odom_pub = create_publisher<nav_msgs::msg::Odometry>("/radar_graph_slam/aftmapped_odom", 16);
    aftmapped_odom_incremenral_pub = create_publisher<nav_msgs::msg::Odometry>("/radar_graph_slam/aftmapped_odom_incremental", 16);
    rclcpp::QoS latched_qos(1); latched_qos.transient_local();
    map_points_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/radar_graph_slam/map_points", latched_qos);
    read_until_pub = create_publisher<std_msgs::msg::Header>("/radar_graph_slam/read_until", 32);
    odom_frame2frame_pub = create_publisher<nav_msgs::msg::Odometry>("/radar_graph_slam/odom_frame2frame", 16);

    map2base_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    dump_service_server = create_service<radar_graph_slam::srv::DumpGraph>(
      "/radar_graph_slam/dump", 
      std::bind(&RadarGraphSlamComponent::dump_service, 
        this, 
        std::placeholders::_1, 
        std::placeholders::_2)
      );
    save_map_service_server = create_service<radar_graph_slam::srv::SaveMap>(
      "/radar_graph_slam/save_map", 
      std::bind(&RadarGraphSlamComponent::save_map_service, 
        this, 
        std::placeholders::_1, 
        std::placeholders::_2)
      );

    graph_updated = false;
    double graph_update_interval = declare_parameter<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = declare_parameter<double>("map_cloud_update_interval", 10.0);
    optimization_timer = create_wall_timer(
      std::chrono::duration<double>(graph_update_interval),
      std::bind(&RadarGraphSlamComponent::optimization_timer_callback, this)
    );
    map_publish_timer = create_wall_timer(
      std::chrono::duration<double>(map_cloud_update_interval), 
      std::bind(&RadarGraphSlamComponent::map_points_publish_timer_callback, this)
    );

    fix_first_node = declare_parameter<bool>("fix_first_node", false);
    fix_first_node_stddev = declare_parameter<std::string>("fix_first_node_stddev", "1 1 1 1 1 1");
    odom_edge_robust_kernel = declare_parameter<std::string>("odometry_edge_robust_kernel", "NONE");
    odom_edge_robust_size  = declare_parameter<double>("odometry_edge_robust_kernel_size", 1.0);
    integ_edge_robust_kernel = declare_parameter<std::string>("integ_edge_robust_kernel", "NONE");
    integ_edge_robust_size = declare_parameter<double>("integ_edge_robust_kernel_size", 1.0);
    enable_loop_closure = declare_parameter<bool>("enable_loop_closure", false);
    fix_first_node_adaptive = declare_parameter<bool>("fix_first_node_adaptive", true);
    g2o_solver_num_iterations = declare_parameter<int>("g2o_solver_num_iterations", 1024);
    gps_edge_robust_kernel = declare_parameter<std::string>("gps_edge_robust_kernel", "NONE");
    gps_edge_robust_kernel_size = declare_parameter<double>("gps_edge_robust_kernel_size", 1.0);
    barometer_edge_robust_kernel = declare_parameter<std::string>("barometer_edge_robust_kernel", "NONE");
    barometer_edge_robust_kernel_size = declare_parameter<double>("barometer_edge_robust_kernel_size", 1.0);
    loop_closure_edge_robust_kernel = declare_parameter<std::string>("loop_closure_edge_robust_kernel", "NONE");
    loop_closure_edge_robust_kernel_size = declare_parameter<double>("loop_closure_edge_robust_kernel_size", 1.0);
  
    if (dataset_name == "loop3")
    utm_to_world << 
     -0.057621,       0.996222,      -0.064972, -128453.624105,
     -0.998281,      -0.058194,      -0.006954,  361869.958099,
     -0.010708,       0.064459,       0.997863,   -5882.237973,
      0.000000,       0.000000,       0.000000,       1.000000;
    else if (dataset_name == "loop2")
    utm_to_world <<
     -0.085585,       0.995774,      -0.033303, -117561.214476,
     -0.996323,      -0.085401,       0.006904,  364927.287181,
      0.004031,       0.033772,       0.999421,   -6478.377842,
      0.000000,       0.000000,       0.000000,       1.000000;

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        init_timer_->cancel();
        keyframe_updater.reset(new KeyframeUpdater(shared_from_this()));
        loop_detector.reset(new LoopDetector(shared_from_this()));
        inf_calclator.reset(new InformationMatrixCalculator(shared_from_this()));
        registration = select_registration_method(shared_from_this());
      }
    );
  }


private:
  rclcpp::TimerBase::SharedPtr init_timer_;
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(OdomMsg::ConstSharedPtr odom_msg, CloudMsg::ConstSharedPtr cloud_msg) {
    const rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Isometry3d odom_now = odom2isometry(odom_msg);
    Eigen::Matrix4d matrix_map2base;
    // Publish TF between /map and /base_link
    if(keyframes.size() > 0)
    {
      const KeyFrame::Ptr& keyframe_last = keyframes.back();
      Eigen::Isometry3d lastkeyframe_odom_incre =  keyframe_last->odom_scan2scan.inverse() * odom_now;
      Eigen::Isometry3d keyframe_map2base_matrix = keyframe_last->node->estimate();
      // map2base = odom^(-1) * base
      matrix_map2base = (keyframe_map2base_matrix * lastkeyframe_odom_incre).matrix();
    }
    geometry_msgs::msg::TransformStamped map2base_trans = matrix2transform(cloud_msg->header.stamp, matrix_map2base, mapFrame, baselinkFrame);
    if (pow(map2base_trans.transform.rotation.w,2)+pow(map2base_trans.transform.rotation.x,2)+
      pow(map2base_trans.transform.rotation.y,2)+pow(map2base_trans.transform.rotation.z,2) < pow(0.9,2)) 
      {map2base_trans.transform.rotation.w=1; map2base_trans.transform.rotation.x=0; map2base_trans.transform.rotation.y=0; map2base_trans.transform.rotation.z=0;}
    map2base_broadcaster->sendTransform(map2base_trans);
   
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(baselinkFrame.empty()) {
      baselinkFrame = cloud_msg->header.frame_id;
    }
    
    // Push ego velocity to queue
    auto twist_ = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_->header.stamp = cloud_msg->header.stamp;
    twist_->twist.twist.linear = odom_msg->twist.twist.linear;
    {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      twist_queue.push_back(twist_);
    }

    //********** Decided whether to accept the frame as a key frame or not **********
    if(!keyframe_updater->decide(odom_now, stamp)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::msg::Header read_until;
        read_until.stamp = (stamp + rclcpp::Duration::from_seconds(10.0));
        read_until.frame_id = points_topic;
        read_until_pub->publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub->publish(read_until);
      }
      return;
    }
    // Get time of this key frame for Intergeration, to integerate between two key frames
    thisKeyframeTime = rclcpp::Time(cloud_msg->header.stamp).seconds();
    
    double accum_d = keyframe_updater->get_accum_distance();
    // Construct keyframe
    KeyFrame::Ptr keyframe(new KeyFrame(keyframe_index, stamp, odom_now, accum_d, cloud));
    keyframe_index ++;

    if (enable_preintegration){
      // Intergerate translation of ego velocity, add rotation
      geometry_msgs::msg::Transform transf_integ = preIntegrationTransform();
      static uint32_t sequ = 0;
      nav_msgs::msg::Odometry odom_frame2frame;
      odom_frame2frame.pose.pose.orientation = transf_integ.rotation;
      odom_frame2frame.pose.pose.position.x = transf_integ.translation.x;
      odom_frame2frame.pose.pose.position.y = transf_integ.translation.y;
      odom_frame2frame.pose.pose.position.z = transf_integ.translation.z;
      odom_frame2frame.header.frame_id = "map";
      odom_frame2frame.header.stamp = cloud_msg->header.stamp;
      odom_frame2frame_pub->publish(odom_frame2frame);
      keyframe->trans_integrated = transf_integ;
    }

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);

    // Scan Context loop detector - giseop
    // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
    // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
    // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
    const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

    if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
        loop_detector->scManager->makeAndSaveScancontextAndKeys(*cloud);
    }
    // else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) { 
    //     scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
    // }
    // else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
    //     pcl::PointCloud<PointT>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointT>());
    //     loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum);
    //     scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
    // }
    

    lastKeyframeTime = thisKeyframeTime;
  }


  void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr imu_odom_msg) {
    // Transform to Radar's Frame
    auto imu_quat = std::make_shared<geometry_msgs::msg::QuaternionStamped>();
    imu_quat->quaternion = imu_odom_msg->orientation;
    Eigen::Quaterniond imu_quat_from(imu_quat->quaternion.w, imu_quat->quaternion.x, imu_quat->quaternion.y, imu_quat->quaternion.z);
    Eigen::Quaterniond imu_quat_deskew = imu_quat_from * extQRPY;
    imu_quat_deskew.normalize();

    static int cnt = 0;
    if(cnt == 0) {
      double roll, pitch, yaw;
      tf2::Quaternion orientation(imu_quat_deskew.x(), imu_quat_deskew.y(), imu_quat_deskew.z(), imu_quat_deskew.w());
      tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      // Eigen::Matrix3d imu_mat_deskew = imu_quat_deskew.toRotationMatrix();
      // Eigen::Vector3d eulerAngle = imu_mat_deskew.eulerAngles(0,1,2); // roll pitch yaw
      Eigen::AngleAxisd rollAngle(AngleAxisd(roll,Vector3d::UnitX()));
      Eigen::AngleAxisd pitchAngle(AngleAxisd(pitch,Vector3d::UnitY()));
      Eigen::AngleAxisd yawAngle(AngleAxisd(0.0,Vector3d::UnitZ()));
      Eigen::Matrix3d imu_mat_final; imu_mat_final = yawAngle * pitchAngle * rollAngle;

      Eigen::Isometry3d isom_initial_pose;
      isom_initial_pose.setIdentity();
      isom_initial_pose.rotate(imu_mat_final); // Set rotation
      initial_pose = isom_initial_pose.matrix();
      RCLCPP_INFO(get_logger(), "Initial Position Matrix = ");
      std::cout << 
        initial_pose(0,0) << ", " << initial_pose(0,1) << ", " << initial_pose(0,2) << ", " << initial_pose(0,3) << ", " << std::endl <<
        initial_pose(1,0) << ", " << initial_pose(1,1) << ", " << initial_pose(1,2) << ", " << initial_pose(1,3) << ", " << std::endl <<
        initial_pose(2,0) << ", " << initial_pose(2,1) << ", " << initial_pose(2,2) << ", " << initial_pose(2,3) << ", " << std::endl <<
        initial_pose(3,0) << ", " << initial_pose(3,1) << ", " << initial_pose(3,2) << ", " << initial_pose(3,3) << ", " << std::endl << std::endl;
      cnt = 1;
    }
  }

  void imu_odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr imu_odom_msg) {
    {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    imu_odom_queue.push_back(imu_odom_msg);
    }
  }

  geometry_msgs::msg::Transform preIntegrationTransform(){
    double lastImuOdomQT = -1;
    // double lastTwistQT = -1;
    double delta_t = 0;
    size_t imu_odom_end_index = 0; // The index of the last used IMU odom
    geometry_msgs::msg::Transform trans_; // Transform to return
    // pop old IMU orientation message
    while (!imu_odom_queue.empty() && rclcpp::Time(imu_odom_queue.front()->header.stamp).seconds() < lastKeyframeTime - delta_t){
        lastImuOdomQT = rclcpp::Time(imu_odom_queue.front()->header.stamp).seconds();
        imu_odom_queue.pop_front();
    }
    // pop old Twist message
    while (!twist_queue.empty() && rclcpp::Time(twist_queue.front()->header.stamp).seconds() < lastKeyframeTime - delta_t){
        // lastTwistQT = twist_queue.front()->header.stamp.toSec();
        twist_queue.pop_front();
    }
    // repropogate
    Eigen::Isometry3d isom_frame2frame;  // Translation increment between last key frame and this IMU msg
    Eigen::Isometry3d isometry_rotation; // Rotation of IMU odom
    Eigen::Isometry3d isometry_translation;  // Translation of IMU odom
    isom_frame2frame.setIdentity();
    isometry_rotation.setIdentity();
    geometry_msgs::msg::Vector3 translation_frame2frame; // Translation between two key frames
    if (!imu_odom_queue.empty())
    {
      // (Doing this because some imu_odom later than thisKeyframeTime will be recieved)
      for (size_t i = 0; i < imu_odom_queue.size(); ++i){
        if (rclcpp::Time(imu_odom_queue.at(i)->header.stamp).seconds() < thisKeyframeTime)
          imu_odom_end_index = i;
      }
      std::unique_ptr<CubicInterpolation> egovelIntegratorX;
      std::unique_ptr<CubicInterpolation> egovelIntegratorY;
      std::unique_ptr<CubicInterpolation> egovelIntegratorZ;
      if (use_egovel_preinteg_trans) {
        // integrate imu message from the beginning of this optimization
        size_t twist_length = twist_queue.size();
        Eigen::MatrixXd q_via_x(twist_length,3),q_via_y(twist_length,3),q_via_z(twist_length,3);
        Eigen::VectorXd t_via_x(twist_length),t_via_y(twist_length),t_via_z(twist_length);
        // double dt = 1.0/12;
        for (size_t i=0; i < twist_length; i++){
          q_via_x(i, 0) = twist_queue.at(i)->twist.twist.linear.x;  q_via_x(i, 1) = 0;  q_via_x(i, 2) = 0;
          // acc: q(i,1)=(twist_queue.at(i+1).twist.linear.x - twist_queue.at(i).twist.linear.x) / dt;
          t_via_x(i) = rclcpp::Time(twist_queue.at(i)->header.stamp).seconds();
          q_via_y(i, 0) = twist_queue.at(i)->twist.twist.linear.y;  q_via_y(i, 1) = 0;  q_via_y(i, 2) = 0;
          t_via_y(i) = t_via_x(i);
          q_via_z(i, 0) = twist_queue.at(i)->twist.twist.linear.z;  q_via_z(i, 1) = 0;  q_via_z(i, 2) = 0;
          t_via_z(i) = t_via_x(i);
        }
        egovelIntegratorX.reset(new CubicInterpolation(q_via_x, t_via_x)); // Integrator of ego velocity at X axis
        egovelIntegratorY.reset(new CubicInterpolation(q_via_y, t_via_y)); // Integrator of ego velocity at Y axis
        egovelIntegratorZ.reset(new CubicInterpolation(q_via_z, t_via_z)); // Integrator of ego velocity at Z axis
        // ROS_INFO("Twist queue - Start time: %f Stop time: %f", t_via_x(0), t_via_x(t_via_x.size()-1));
      }
      Eigen::Isometry3d isom_imu_odom_last(Eigen::Isometry3d::Identity());
      Eigen::Quaterniond q_imu_odom(imu_odom_queue.front()->pose.pose.orientation.w,
                                imu_odom_queue.front()->pose.pose.orientation.x,
                                imu_odom_queue.front()->pose.pose.orientation.y,
                                imu_odom_queue.front()->pose.pose.orientation.z);
      isom_imu_odom_last.rotate(q_imu_odom);
      isom_imu_odom_last.pretranslate(Eigen::Vector3d(imu_odom_queue.front()->pose.pose.position.x,
                                                  imu_odom_queue.front()->pose.pose.position.y,
                                                  imu_odom_queue.front()->pose.pose.position.z));
      Eigen::Isometry3d isom_imu_odom_this(Eigen::Isometry3d::Identity());
      q_imu_odom = Eigen::Quaterniond(imu_odom_queue.at(imu_odom_end_index)->pose.pose.orientation.w,
                                imu_odom_queue.at(imu_odom_end_index)->pose.pose.orientation.x,
                                imu_odom_queue.at(imu_odom_end_index)->pose.pose.orientation.y,
                                imu_odom_queue.at(imu_odom_end_index)->pose.pose.orientation.z);
      isom_imu_odom_this.rotate(q_imu_odom);
      isom_imu_odom_this.pretranslate(Eigen::Vector3d(imu_odom_queue.at(imu_odom_end_index)->pose.pose.position.x,
                                                  imu_odom_queue.at(imu_odom_end_index)->pose.pose.position.y,
                                                  imu_odom_queue.at(imu_odom_end_index)->pose.pose.position.z));
      Eigen::Isometry3d isom_imu_odom_btn = isom_imu_odom_last.inverse() * isom_imu_odom_this;
      // cout << isom_imu_odom_btn.matrix() << endl;
      // ROS_INFO("IMU Odom queue - Start time: %f Stop time: %f", imu_odom_queue.at(0)->header.stamp.toSec(), imu_odom_queue.at(imu_odom_end_index)->header.stamp.toSec());
      if (use_egovel_preinteg_trans){ // Use Ego vel as translation integration
        for (size_t i = 0; i < imu_odom_end_index + 1; i++)
        {
            nav_msgs::msg::Odometry::ConstSharedPtr thisImu = imu_odom_queue.at(i);
            double imuOdomTime = rclcpp::Time(thisImu->header.stamp).seconds();
            isometry_translation.setIdentity();
            isometry_rotation.setIdentity();
            
            double dt = (lastImuOdomQT < 0) ? (1.0 / 200.0) :(imuOdomTime - lastImuOdomQT);
            Eigen::Vector3d translation_ = Eigen::Vector3d( egovelIntegratorX->getPosition(imuOdomTime),
                                                            egovelIntegratorY->getPosition(imuOdomTime),
                                                            egovelIntegratorZ->getPosition(imuOdomTime))* dt;
            isometry_translation.pretranslate(translation_); // Set translation
            isom_frame2frame = isom_frame2frame.pretranslate(isometry_translation.translation());

            lastImuOdomQT = imuOdomTime;
        }
      }
      else {  // Use IMU as translation integration
        isom_frame2frame = isom_imu_odom_btn;
      }
      translation_frame2frame.x = isom_frame2frame.translation().x();
      translation_frame2frame.y = isom_frame2frame.translation().y();
      translation_frame2frame.z = isom_frame2frame.translation().z();
      const auto& last_imu_orien = imu_odom_queue.at(imu_odom_end_index);
      trans_.rotation = last_imu_orien->pose.pose.orientation;
      trans_.translation = translation_frame2frame;
    }
    return trans_;
  }


  void barometer_callback(barometer_bmp388::msg::Barometer::ConstSharedPtr baro_msg) {
    if(!enable_barometer) {
      return;
    }
    std::lock_guard<std::mutex> lock(barometer_queue_mutex);
    barometer_queue.push_back(baro_msg);
  }

  bool flush_barometer_queue() {
    std::lock_guard<std::mutex> lock(barometer_queue_mutex);
    if(keyframes.empty() || barometer_queue.empty()) {
      return false;
    }
    bool updated = false;
    auto barometer_cursor = barometer_queue.begin();

    for(size_t i=0; i < keyframes.size(); i++) {
      auto keyframe = keyframes.at(i);
      if(keyframe->stamp > barometer_queue.back()->header.stamp) {
        break;
      }
      if(keyframe->stamp < (*barometer_cursor)->header.stamp || keyframe->altitude) {
        continue;
      }
      // find the barometer data which is closest to the keyframe_
      auto closest_barometer = barometer_cursor;
      for(auto baro = barometer_cursor; baro != barometer_queue.end(); baro++) {
        auto dt  = (rclcpp::Time((*closest_barometer)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*baro)->header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }
        closest_barometer = baro;
      }
      // if the time residual between the barometer and keyframe_ is too large, skip it
      barometer_cursor = closest_barometer;
      if(0.2 < std::abs((rclcpp::Time((*closest_barometer)->header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      Eigen::Vector4d aftmapped_pos(keyframe->odom_scan2scan.translation().x(), keyframe->odom_scan2scan.translation().y(), (*closest_barometer)->altitude, 1.0);
      aftmapped_pos = initial_pose * aftmapped_pos;
      // std::cout << "baro position: " << aftmapped_pos(0) << ", " << aftmapped_pos(1) << ", " << aftmapped_pos(2) << std::endl;
      
      nav_msgs::msg::Odometry initial_odom; // Initial posture
      initial_odom = matrix2odom(keyframe->stamp, initial_pose, mapFrame, baselinkFrame);

      Eigen::Vector1d z(aftmapped_pos(2));
      // the first barometer data position will be the origin of the map
      if(!zero_alt) {
        zero_alt = z;
      }
      z -= (*zero_alt);
      keyframe->altitude = z;

      if (enable_barometer){ 
        //********** G2O Edge *********** 
        if (barometer_edge_type == 1){
          g2o::OptimizableGraph::Edge* edge;
          Eigen::Vector1d information_matrix = Eigen::Vector1d::Identity() / barometer_edge_stddev;
          edge = graph_slam->add_se3_prior_z_edge(keyframe->node, z, information_matrix);
          graph_slam->add_robust_kernel(edge, barometer_edge_robust_kernel, barometer_edge_robust_kernel_size);
        }
        else if (barometer_edge_type == 2 && i != 0 && keyframes.at(i-1)->altitude.is_initialized()){
          g2o::OptimizableGraph::Edge* edge;
          Eigen::Vector1d information_matrix = Eigen::Vector1d::Identity() / barometer_edge_stddev;
          Eigen::Vector1d relative_z(keyframe->altitude.value() - keyframes.at(i-1)->altitude.value());
          edge = graph_slam->add_se3_z_edge(keyframe->node, keyframes.at(i-1)->node, relative_z, information_matrix);
          graph_slam->add_robust_kernel(edge, barometer_edge_robust_kernel, barometer_edge_robust_kernel_size);
        }
      }
      updated = true;
    }
    auto remove_loc = std::upper_bound(barometer_queue.begin(), barometer_queue.end(), keyframes.back()->stamp, [=](const rclcpp::Time& stamp, const barometer_bmp388::msg::Barometer::ConstSharedPtr& baropoint) { return stamp < rclcpp::Time(baropoint->header.stamp); });
    barometer_queue.erase(barometer_queue.begin(), remove_loc);
    return updated;
  }

  /**
   * @brief this method adds all the keyframes_ in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe_ was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    // ********** Select number of keyframess to be optimized **********
    for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframess will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom_scan2scan;
      // ********** Vertex of keyframess is contructed here ***********
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(fix_first_node) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::stringstream sst(fix_first_node_stddev);
          for(int i = 0; i < 6; i++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(i, i) = 1.0 / stddev;
          }
          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }
      
      if(i == 0 && keyframes.empty()) {
        continue;
      }

      /***** Scan-to-Scan Add edge to between consecutive keyframes *****/
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];
      // relative pose between odom of previous frame and this frame R2=R12*R1 => R12 = inv(R2) * R1
      Eigen::Isometry3d relative_pose = keyframe->odom_scan2scan.inverse() * prev_keyframe->odom_scan2scan;
      // calculate fitness score as information 
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      // cout << information << endl;
      graph_slam->add_robust_kernel(edge, odom_edge_robust_kernel, odom_edge_robust_size);

      if (enable_preintegration){
        // Add Preintegration edge
        geometry_msgs::msg::Transform relative_trans = keyframe->trans_integrated;
        g2o::SE3Quat relative_se3quat ( Eigen::Quaterniond(relative_trans.rotation.w, relative_trans.rotation.x, relative_trans.rotation.y, relative_trans.rotation.z), 
                                        Eigen::Vector3d(relative_trans.translation.x, relative_trans.translation.y, relative_trans.translation.z));
        Eigen::Isometry3d relative_isometry = transform2isometry(relative_trans);
        Eigen::MatrixXd information_integ = Eigen::MatrixXd::Identity(6, 6);
        information_integ <<  1.0 / preinteg_trans_stddev, 0, 0, 0, 0, 0,
                              0, 1.0 / preinteg_trans_stddev, 0, 0, 0, 0,
                              0, 0, 1.0 / preinteg_trans_stddev, 0, 0, 0,
                              0, 0, 0, 1.0 / preinteg_orient_stddev, 0, 0,
                              0, 0, 0, 0, 1.0 / preinteg_orient_stddev, 0,
                              0, 0, 0, 0, 0, 1.0 / preinteg_orient_stddev; 
        auto edge_integ = graph_slam->add_se3_edge(prev_keyframe->node, keyframe->node, relative_isometry, information_integ);
        graph_slam->add_robust_kernel(edge_integ, integ_edge_robust_kernel, integ_edge_robust_size);
      }
    }

    std_msgs::msg::Header read_until;
    read_until.stamp = (keyframe_queue[num_processed]->stamp + rclcpp::Duration::from_seconds(10.0));
    read_until.frame_id = points_topic;
    read_until_pub->publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub->publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }


  /**
   * @brief Back-end Optimization. This methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback() {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes_ and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::msg::Header read_until;
      read_until.stamp = (this->now() + rclcpp::Duration::from_seconds(30.0));
      read_until.frame_id = points_topic;
      read_until_pub->publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub->publish(read_until);
    }

    if(!keyframe_updated & !flush_gps_queue() & !flush_barometer_queue()) {
      return;
    }
    
    // loop detection
    if(enable_loop_closure){
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    }

    // Copy "new_keyframes_" to vector  "keyframes_", "new_keyframes_" was used for loop detaction 
    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    if(enable_loop_closure)
      addLoopFactor();

    // move the first node / position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    if(anchor_node && fix_first_node_adaptive) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    // optimize the pose graph
    int num_iterations = g2o_solver_num_iterations;
    clock_t start_ms = clock();
    graph_slam->optimize(num_iterations);
    clock_t end_ms = clock();
    double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
    opt_time.push_back(time_used);

    //********** publish tf **********
    const auto& keyframe = keyframes.back();
    // RadarOdom_to_base = map_to_base * map_to_RadarOdom^(-1)
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom_scan2scan.inverse();
    Eigen::Isometry3d map2base_trans = keyframe->node->estimate();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix();
    // map2base_incremental = map2base_last^(-1) * map2base_this 
    trans_aftmapped_incremental = trans_aftmapped.inverse() * map2base_trans;
    trans_aftmapped = map2base_trans;
    trans_odom2map_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    // Publish After-Mapped Odometry
    nav_msgs::msg::Odometry aft = isometry2odom(keyframe->stamp, trans_aftmapped, mapFrame, odometryFrame);
    aftmapped_odom_pub->publish(aft);

    // Publish After-Mapped Odometry Incrementation
    nav_msgs::msg::Odometry aft_incre = isometry2odom(keyframe->stamp, trans_aftmapped_incremental, mapFrame, odometryFrame);
    aftmapped_odom_incremenral_pub->publish(aft_incre);

    // Publish /odom to /base_link
    if(odom2base_pub->get_subscription_count() > 0) {  // Returns the number of subscribers that are currently connected to this Publisher
      geometry_msgs::msg::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix(), mapFrame, odometryFrame);
      odom2base_pub->publish(ts);
    }

    if(markers_pub->get_subscription_count() > 0) {
      auto markers = create_marker_array(this->now());
      markers_pub->publish(markers);
    }
  }

  void addLoopFactor()
  {
    if (loop_detector->loopIndexQueue.empty())
      return;
    for (int i = 0; i < (int)loop_detector->loopIndexQueue.size(); ++i){
      int indexFrom = loop_detector->loopIndexQueue[i].first;
      int indexTo = loop_detector->loopIndexQueue[i].second;
      Eigen::Isometry3d poseBetween = loop_detector->loopPoseQueue[i];
      Eigen::MatrixXd information_matrix = loop_detector->loopInfoQueue[i];
      auto edge = graph_slam->add_se3_edge(keyframes[indexFrom]->node, keyframes[indexTo]->node, poseBetween, information_matrix);
      graph_slam->add_robust_kernel(edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size);
    }
    // loopIndexQueue.clear();
    // loopPoseQueue.clear();
    // loopInfoQueue.clear();
    // aLoopIsClosed = true;
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback() {
    if(!map_points_pub->get_subscription_count() || !graph_updated) {
      return;
    }
    std::vector<KeyFrameSnapshot::Ptr> snapshot;
    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if(!cloud) {
      return;
    }
    cloud->header.frame_id = mapFrame;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub->publish(*cloud_msg);
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::msg::MarkerArray create_marker_array(const rclcpp::Time& stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    if (show_sphere)
      markers.markers.resize(5);
    else
      markers.markers.resize(4);

    // loop edges
    visualization_msgs::msg::Marker& loop_marker = markers.markers[0];
    loop_marker.header.frame_id = "map";
    loop_marker.header.stamp = stamp;
    loop_marker.action = visualization_msgs::msg::Marker::ADD;
    loop_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    loop_marker.ns = "loop_edges";
    loop_marker.id = 1;
    loop_marker.pose.orientation.w = 1;
    loop_marker.scale.x = 0.1; loop_marker.scale.y = 0.1; loop_marker.scale.z = 0.1;
    loop_marker.color.r = 0.9; loop_marker.color.g = 0.9; loop_marker.color.b = 0;
    loop_marker.color.a = 1;
    for (auto it = loop_detector->loopIndexContainer.begin(); it != loop_detector->loopIndexContainer.end(); ++it)
    {
      int key_cur = it->first;
      int key_pre = it->second;
      geometry_msgs::msg::Point p;
      Eigen::Vector3d pos = keyframes[key_cur]->node->estimate().translation();
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      loop_marker.points.push_back(p);
      pos = keyframes[key_pre]->node->estimate().translation();
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      loop_marker.points.push_back(p);
    }

    // node markers
    visualization_msgs::msg::Marker& traj_marker = markers.markers[1];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.3;

    visualization_msgs::msg::Marker& imu_marker = markers.markers[2];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
    imu_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(size_t i = 0; i < keyframes.size(); i++) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 0.0;//1.0 - p;
      traj_marker.colors[i].g = 1.0;//p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::msg::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::msg::Marker& edge_marker = markers.markers[3];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i * 2].r = 0.0;//1.0 - p1;
        edge_marker.colors[i * 2].g = 1.0;//p1;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 0.0;//1.0 - p2;
        edge_marker.colors[i * 2 + 1].g = 1.0;//p2;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          // edge_marker.points[i * 2].z += 0.5;
          // edge_marker.points[i * 2 + 1].z += 0.5;
          edge_marker.colors[i * 2].r = 0.9;
          edge_marker.colors[i * 2].g = 0.9;
          edge_marker.colors[i * 2].b = 0.0;
          edge_marker.colors[i * 2 + 1].r = 0.9;
          edge_marker.colors[i * 2 + 1].g = 0.9;
          edge_marker.colors[i * 2 + 1].b = 0.0;
          edge_marker.colors[i * 2].a = 0.0;
          edge_marker.colors[i * 2 + 1].a += 0.0;
        }
        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].b = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].b = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }

    if (show_sphere)
    {
      // sphere
      visualization_msgs::msg::Marker& sphere_marker = markers.markers[4];
      sphere_marker.header.frame_id = "map";
      sphere_marker.header.stamp = stamp;
      sphere_marker.ns = "loop_close_radius";
      sphere_marker.id = 3;
      sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;

      if(!keyframes.empty()) {
        Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();
      }
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

      sphere_marker.color.r = 1.0;
      sphere_marker.color.a = 0.3;
    }

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  void dump_service(
    const std::shared_ptr<radar_graph_slam::srv::DumpGraph::Request> req, 
    std::shared_ptr<radar_graph_slam::srv::DumpGraph::Response> res) {
    
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req->destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(size_t i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
    ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

    res->success = true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  void save_map_service(
    const std::shared_ptr<radar_graph_slam::srv::SaveMap::Request> req, 
    std::shared_ptr<radar_graph_slam::srv::SaveMap::Response> res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req->resolution);
    if(!cloud) {
      res->success = false;
      return;
    }

    if(zero_utm && req->utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = mapFrame;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req->destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req->destination, *cloud);
    res->success = ret == 0;
  }

  void nmea_callback(nmea_msgs::msg::Sentence::ConstSharedPtr nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);
    if(grmc.status != 'A')
      return;
    auto gps_msg = std::make_shared<geographic_msgs::msg::GeoPointStamped>();
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;
    gps_callback(gps_msg);
  }

  void navsat_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr navsat_msg) {
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header = navsat_msg->header;
    gps_msg.latitude = navsat_msg->latitude;
    gps_msg.longitude = navsat_msg->longitude;
    gps_msg.altitude = navsat_msg->altitude;
    gps_msg.position_covariance = navsat_msg->position_covariance;
    gps_msg.position_covariance_type = navsat_msg->position_covariance_type;
    gps_msg.status = navsat_msg->status;
    gps_navsat_queue.push_back(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue_
   * @param gps_msg
   */
  void gps_callback(geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    rclcpp::Time t(gps_msg->header.stamp);
    gps_msg->header.stamp = (t + rclcpp::Duration::from_seconds(gps_time_offset));
    gps_geopoint_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_navsat_queue.empty()) {
      return false;
    }
    
    bool updated = false;
    auto gps_cursor = gps_navsat_queue.begin();

    for(auto& keyframe : keyframes) {
      if (keyframe->index - last_gps_edge_index < gps_edge_intervals) continue;
      if (keyframe->stamp > gps_navsat_queue.back().header.stamp) {
        break;
      }
      if (keyframe->stamp < (*gps_cursor).header.stamp || keyframe->utm_coord) {
        continue;
      }
      // find the gps data which is closest to the keyframe_
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_navsat_queue.end(); gps++) {
        auto dt  = (rclcpp::Time((*closest_gps).header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*gps).header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }
        closest_gps = gps;
      }
      // if the time residual between the gps and keyframe_ is too large, skip it
      gps_cursor = closest_gps;
      if(0.2 < std::abs((rclcpp::Time((*closest_gps).header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      geographic_msgs::msg::GeoPoint gps_geopoint;
      gps_geopoint.altitude = (*closest_gps).altitude;
      gps_geopoint.latitude = (*closest_gps).latitude;
      gps_geopoint.longitude = (*closest_gps).longitude;
      geodesy::UTMPoint utm;
      geodesy::fromMsg(gps_geopoint, utm); 
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
      double cov_x = (*closest_gps).position_covariance.at(0);
      double cov_y = (*closest_gps).position_covariance.at(4);
      double cov_z = (*closest_gps).position_covariance.at(8);
      if (cov_x > max_gps_edge_stddev_xy || cov_y > max_gps_edge_stddev_xy || cov_z > max_gps_edge_stddev_z)
        continue;

      // the first gps data position will be the origin of the map
      // if(!zero_utm) {
      //   zero_utm = xyz;
      // }
      // xyz -= (*zero_utm);
      keyframe->utm_coord = xyz;
      Eigen::Vector3d world_coordinate = (utm_to_world * Eigen::Vector4d(utm.easting, utm.northing, utm.altitude, 1)).head(3);
      Eigen::Vector3d trans_err = keyframe->node->estimate().translation() - world_coordinate;
      if (trans_err.norm() < 5.0) continue;
      //********** G2O Edge ***********
      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(world_coordinate.z())) {
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / cov_x;
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, world_coordinate.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix(0, 0) /= cov_x;
        information_matrix(1, 1) /= cov_y;
        information_matrix(2, 2) /= cov_z;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, world_coordinate, information_matrix);
      }
      graph_slam->add_robust_kernel(edge, gps_edge_robust_kernel, gps_edge_robust_kernel_size);

      last_gps_edge_index = keyframe->index;
      updated = true;
    }
    
    auto remove_loc = std::upper_bound(gps_navsat_queue.begin(), gps_navsat_queue.end(), keyframes.back()->stamp, [=](const rclcpp::Time& stamp, const sensor_msgs::msg::NavSatFix& geopoint) { return stamp < rclcpp::Time(geopoint.header.stamp); });
    gps_navsat_queue.erase(gps_navsat_queue.begin(), remove_loc);
    
    return updated;
  }
  
  void command_callback(const std_msgs::msg::String& str_msg) {
    if (str_msg.data == "output_aftmapped") {
      ofstream fout;
      fout.open("/home/zhuge/stamped_pose_graph_estimate.txt", ios::out);
      fout << "# timestamp tx ty tz qx qy qz qw" << endl;
      fout.setf(ios::fixed, ios::floatfield);  // fixed mode，float
      fout.precision(8);  // Set precision 8
      for(size_t i = 0; i < keyframes.size(); i++) {
        Eigen::Vector3d pos_ = keyframes[i]->node->estimate().translation();
        Eigen::Matrix3d rot_ = keyframes[i]->node->estimate().rotation();
        Eigen::Quaterniond quat_(rot_);
        double timestamp = keyframes[i]->stamp.seconds();
        double tx = pos_(0), ty = pos_(1), tz = pos_(2);
        double qx = quat_.x(), qy = quat_.y(), qz = quat_.z(), qw = quat_.w();

        fout << timestamp << " "
          << tx << " " << ty << " " << tz << " "
          << qx << " " << qy << " " << qz << " " << qw << endl;
      }
      fout.close();
      RCLCPP_INFO(get_logger(), "Optimized edges have been output!");
    }
    else if (str_msg.data == "time") {
      if (loop_detector->pf_time.size() > 0) {
        std::sort(loop_detector->pf_time.begin(), loop_detector->pf_time.end());
        double median = loop_detector->pf_time.at(floor((double)loop_detector->pf_time.size() / 2));
        cout << "Pre-filtering Matching time cost (median): " << median << endl;
      }
      if (loop_detector->sc_time.size() > 0) {
        std::sort(loop_detector->sc_time.begin(), loop_detector->sc_time.end());
        double median = loop_detector->sc_time.at(floor((double)loop_detector->sc_time.size() / 2));
        cout << "Scan Context time cost (median): " << median << endl;
      }
      if (loop_detector->oc_time.size() > 0) {
        std::sort(loop_detector->oc_time.begin(), loop_detector->oc_time.end());
        double median = loop_detector->oc_time.at(floor((double)loop_detector->oc_time.size() / 2));
        cout << "Odometry Check time cost (median): " << median << endl;
      }
      if (opt_time.size() > 0) {
        std::sort(opt_time.begin(), opt_time.end());
        double median = opt_time.at(floor((double)opt_time.size() / 2));
        cout << "Optimization time cost (median): " << median << endl;
      }
    }
  }


private:
  // ROS
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;

  std::shared_ptr<message_filters::Subscriber<OdomMsg>> odom_sub;
  std::shared_ptr<message_filters::Subscriber<CloudMsg>> cloud_sub;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  rclcpp::Subscription<barometer_bmp388::msg::Barometer>::SharedPtr barometer_sub;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4d trans_odom2map; // keyframe->node->estimate() * keyframe->odom.inverse();
  Eigen::Isometry3d trans_aftmapped;  // Odometry from /map to /base_link
  Eigen::Isometry3d trans_aftmapped_incremental;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2base_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aftmapped_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aftmapped_odom_incremenral_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_frame2frame_pub;

  std::string points_topic;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> map2base_broadcaster; // odom_frame => base_frame

  rclcpp::Service<radar_graph_slam::srv::DumpGraph>::SharedPtr dump_service_server;
  rclcpp::Service<radar_graph_slam::srv::SaveMap>::SharedPtr save_map_service_server; 

  // keyframe queue
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> twist_queue;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> imu_odom_queue;
  double thisKeyframeTime;
  double lastKeyframeTime;
  size_t keyframe_index = 0;

  // IMU / Ego Velocity Integration
  bool enable_preintegration;
  double preinteg_orient_stddev;
  double preinteg_trans_stddev;
  bool enable_imu_orientation;
  bool use_egovel_preinteg_trans;
  Eigen::Matrix4d initial_pose;

  // barometer queue
  bool enable_barometer;
  int barometer_edge_type;
  double barometer_edge_stddev;
  boost::optional<Eigen::Vector1d> zero_alt;
  std::mutex barometer_queue_mutex;
  std::deque<barometer_bmp388::msg::Barometer::ConstSharedPtr> barometer_queue;

  // gps queue
  int gps_edge_intervals;
  int last_gps_edge_index;
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  double max_gps_edge_stddev_xy;
  double max_gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::msg::GeoPointStamped::ConstSharedPtr> gps_geopoint_queue;
  std::deque<sensor_msgs::msg::NavSatFix>           gps_navsat_queue;
  Eigen::Matrix4d utm_to_world;
  std::string dataset_name;

  // Marker coefficients
  bool show_sphere;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  //  Used for Loop Closure detection source, 
  //  pushed form keyframe_queue at "flush_keyframe_queue()", 
  //  inserted to "keyframes_" before optimization
  std::deque<KeyFrame::Ptr> new_keyframes;
  //  Previous keyframes_
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;
  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

  // Registration Method
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::KdTreeFLANN<PointT>::Ptr kdtreeHistoryKeyPoses;

  std::vector<double> opt_time;

  bool fix_first_node;
  std::string fix_first_node_stddev;
  std::string odom_edge_robust_kernel;
  double odom_edge_robust_size;
  std::string integ_edge_robust_kernel;
  double integ_edge_robust_size;
  bool enable_loop_closure;
  bool fix_first_node_adaptive;
  int g2o_solver_num_iterations;
  std::string gps_edge_robust_kernel;
  double gps_edge_robust_kernel_size;
  std::string barometer_edge_robust_kernel;
  double barometer_edge_robust_kernel_size;
  std::string loop_closure_edge_robust_kernel;
  double loop_closure_edge_robust_kernel_size;  
};

}  // namespace radar_graph_slam

RCLCPP_COMPONENTS_REGISTER_NODE(radar_graph_slam::RadarGraphSlamComponent)
