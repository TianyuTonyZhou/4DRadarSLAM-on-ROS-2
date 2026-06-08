// SPDX-License-Identifier: BSD-2-Clause
#include <string>
#include <fstream>
#include <functional>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/filter.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include "radar_ego_velocity_estimator.h"
#include "rio_utils/radar_point_cloud.h"
#include "utility_radar.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;

namespace radar_graph_slam {

class PreprocessingComponent : public rclcpp::Node, public ParamServer {
public: 
  // typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZI PointT;


  explicit PreprocessingComponent(const rclcpp::NodeOptions& options)
  : Node("preprocessing", options), ParamServer(this)
  {
    initializeTransformation();

    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointCloudTopic, 64,
      std::bind(&PreprocessingComponent::cloud_callback, this, std::placeholders::_1)
    );
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      imuTopic,
      1,
      std::bind(&PreprocessingComponent::imu_callback, this, std::placeholders::_1)
    );
    command_sub = create_subscription<std_msgs::msg::String>(
      "/command",
      10,
      std::bind(&PreprocessingComponent::command_callback, this, std::placeholders::_1)
    );

    points_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_points",
      32
    );
    colored_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/colored_points",
      32
    );
    imu_pub = create_publisher<sensor_msgs::msg::Imu>(
      "/imu",
      32
    );
    gt_pub = create_publisher<nav_msgs::msg::Odometry>(
      "/aftmapped_to_init",
      16
    );
  
    topic_twist = declare_parameter<std::string>("topic_twist", "/eagle_data/twist");
    topic_inlier_pc2 = declare_parameter<std::string>("topic_inlier_pc2", "/eagle_data/inlier_pc2");
    topic_outlier_pc2 = declare_parameter<std::string>("topic_outlier_pc2", "/eagle_data/outlier_pc2");

    pubtwist = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      topic_twist,
      5
    );

    pubinlier_pc2 = create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_inlier_pc2,
      5
    );

    puboutlier_pc2 = create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_outlier_pc2,
      5
    );

    pc2_raw_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/eagle_data/pc2_raw",
      1
    );

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    enable_dynamic_object_removal = declare_parameter<bool>("enable_dynamic_object_removal", false);
    power_threshold = declare_parameter<double>("power_threshold", 0);
    scan_period = declare_parameter<double>("scan_period", 0.1);
    downsample_method = declare_parameter<std::string>("downsample_method", "VOXELGRID");
    downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    outlier_removal_method = declare_parameter<std::string>("outlier_removal_method", "STATISTICAL");
    statistical_mean_k = declare_parameter<int>("statistical_mean_k", 20);
    statistical_stddev = declare_parameter<double>("statistical_stddev", 1.0);
    radius_radius = declare_parameter<double>("radius_radius", 0.8);
    radius_min_neighbors = declare_parameter<int>("radius_min_neighbors", 2);
    use_distance_filter = declare_parameter<bool>("use_distance_filter", true);
    use_azimuth_filter = declare_parameter<bool>("use_azimuth_filter", false);
    scan_azimuth_min_deg = declare_parameter<double>("scan_azimuth_min_deg", -180.0);
    scan_azimuth_max_deg = declare_parameter<double>("scan_azimuth_max_deg", 180.0);
    distance_near_thresh = declare_parameter<double>("distance_near_thresh", 1.0);
    distance_far_thresh = declare_parameter<double>("distance_far_thresh", 100.0);
    z_low_thresh = declare_parameter<double>("z_low_thresh", -5.0);
    z_high_thresh = declare_parameter<double>("z_high_thresh", 20.0);
    gt_file_location = declare_parameter<std::string>("gt_file_location", "");
    publish_tf = declare_parameter<bool>("publish_tf", false);

    initializeParams();

  }
  
private:
  void initializeTransformation(){
    livox_to_RGB = (cv::Mat_<double>(4,4) << 
    -0.006878330000, -0.999969000000, 0.003857230000, 0.029164500000,  
    -7.737180000000E-05, -0.003856790000, -0.999993000000, 0.045695200000,
     0.999976000000, -0.006878580000, -5.084110000000E-05, -0.19018000000,
    0,  0,  0,  1);
    RGB_to_livox =livox_to_RGB.inv();
    Thermal_to_RGB = (cv::Mat_<double>(4,4) <<
    0.9999526089706319, 0.008963747151337641, -0.003798822163962599, 0.18106962419014,  
    -0.008945181135788245, 0.9999481006917174, 0.004876439015823288, -0.04546324090016857,
    0.00384233617405678, -0.004842226763999368, 0.999980894463835, 0.08046453079998771,
    0,0,0,1);
    Radar_to_Thermal = (cv::Mat_<double>(4,4) <<
    0.999665,    0.00925436,  -0.0241851,  -0.0248342,
    -0.00826999, 0.999146,    0.0404891,   0.0958317,
    0.0245392,   -0.0402755,  0.998887,    0.0268037,
    0,  0,  0,  1);
    Change_Radarframe=(cv::Mat_<double>(4,4) <<
    0,-1,0,0,
    0,0,-1,0,
    1,0,0,0,
    0,0,0,1);
    Radar_to_livox=RGB_to_livox*Thermal_to_RGB*Radar_to_Thermal*Change_Radarframe;
    std::cout << "Radar_to_livox = "<< std::endl << " "  << Radar_to_livox << std::endl << std::endl;
  }
  void initializeParams() {

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = statistical_mean_k;
      double stddev_mul_thresh = statistical_stddev;
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = radius_radius;
      int min_neighbors = radius_min_neighbors;
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } 
    // else if (outlier_removal_method == "BILATERAL")
    // {
    //   double sigma_s = declare_parameter<double>("bilateral_sigma_s", 5.0);
    //   double sigma_r = declare_parameter<double>("bilateral_sigma_r", 0.03);
    //   std::cout << "outlier_removal: BILATERAL " << sigma_s << " - " << sigma_r << std::endl;

    //   pcl::FastBilateralFilter<PointT>::Ptr fbf(new pcl::FastBilateralFilter<PointT>());
    //   fbf->setSigmaS (sigma_s);
    //   fbf->setSigmaR (sigma_r);
    //   outlier_removal_filter = fbf;
    // }
     else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    std::string file_name = gt_file_location;

    ifstream file_in(file_name);
    if (!file_in.is_open()) {
        cout << "Can not open this gt file" << endl;
    }
    else{
      std::vector<std::string> vectorLines;
      std::string line;
      while (getline(file_in, line)) {
          vectorLines.push_back(line);
      }
      
      for (size_t i = 1; i < vectorLines.size(); i++) {
          std::string line_ = vectorLines.at(i);
          double stamp,tx,ty,tz,qx,qy,qz,qw;
          stringstream data(line_);
          data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
          nav_msgs::msg::Odometry odom_msg;
          odom_msg.header.frame_id = mapFrame;
          odom_msg.child_frame_id = baselinkFrame;
          odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
          odom_msg.pose.pose.orientation.w = qw;
          odom_msg.pose.pose.orientation.x = qx;
          odom_msg.pose.pose.orientation.y = qy;
          odom_msg.pose.pose.orientation.z = qz;
          odom_msg.pose.pose.position.x = tx;
          odom_msg.pose.pose.position.y = ty;
          odom_msg.pose.pose.position.z = tz;
          std::lock_guard<std::mutex> lock(odom_queue_mutex);
          odom_msgs.push_back(odom_msg);
      }
    }
    file_in.close();
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    sensor_msgs::msg::Imu imu_data;
    imu_data.header.stamp = imu_msg->header.stamp;
    imu_data.header.frame_id = "imu_frame";
    Eigen::Quaterniond q_ahrs(imu_msg->orientation.w,
                              imu_msg->orientation.x,
                              imu_msg->orientation.y,
                              imu_msg->orientation.z);
    Eigen::Quaterniond q_r = 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr = 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = imu_msg->angular_velocity.x;
    imu_data.angular_velocity.y = -imu_msg->angular_velocity.y;
    imu_data.angular_velocity.z = -imu_msg->angular_velocity.z;
    imu_data.linear_acceleration.x = imu_msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = -imu_msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = -imu_msg->linear_acceleration.z;
    imu_pub->publish(imu_data);
    // imu_queue.push_back(imu_msg);
    // double time_now = imu_msg->header.stamp.toSec();
    double time_now = rclcpp::Time(imu_msg->header.stamp).seconds();
    bool updated = false;
    if (odom_msgs.size() != 0) {
      while (rclcpp::Time(odom_msgs.front().header.stamp).seconds() + 0.001 < time_now) {
        std::lock_guard<std::mutex> lock(odom_queue_mutex);
        odom_msgs.pop_front();
        updated = true;
        if (odom_msgs.size() == 0)
          break;
      }
    }
    if (updated == true && odom_msgs.size() > 0){
      if (publish_tf) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.child_frame_id = baselinkFrame;
        tf_msg.header.frame_id = mapFrame;
        tf_msg.header.stamp = odom_msgs.front().header.stamp;
        // tf_msg.header.stamp = ros::Time().now();
        tf_msg.transform.rotation = odom_msgs.front().pose.pose.orientation;
        tf_msg.transform.translation.x = odom_msgs.front().pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msgs.front().pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msgs.front().pose.pose.position.z;
        tf_broadcaster->sendTransform(tf_msg);
      }
      
      gt_pub->publish(odom_msgs.front());
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr eagle_msg) {

    pcl::PointCloud<PointT>::Ptr radarcloud_xyzi(new pcl::PointCloud<PointT>());
    radarcloud_xyzi->header.frame_id = baselinkFrame;
    radarcloud_xyzi->header.stamp = pcl_conversions::toPCL(eagle_msg->header.stamp);

    // Find field offsets
    int x_offset = -1, y_offset = -1, z_offset = -1;
    int snr_offset = -1, vel_offset = -1;
    for (const auto& field : eagle_msg->fields) {
      if (field.name == "x")                    x_offset   = field.offset;
      else if (field.name == "y")               y_offset   = field.offset;
      else if (field.name == "z")               z_offset   = field.offset;
      else if (field.name == "signal_noise_ratio") snr_offset = field.offset;
      else if (field.name == "radial_velocity") vel_offset = field.offset;
    }

    const uint8_t* data = eagle_msg->data.data();
    const uint32_t point_step = eagle_msg->point_step;

    for (uint32_t i = 0; i < eagle_msg->width * eagle_msg->height; i++) {
      const uint8_t* p = data + i * point_step;

      float x   = *reinterpret_cast<const float*>(p + x_offset);
      float y   = *reinterpret_cast<const float*>(p + y_offset);
      float z   = *reinterpret_cast<const float*>(p + z_offset);
      float snr = snr_offset >= 0 ? *reinterpret_cast<const float*>(p + snr_offset) : 0.0f;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (snr < power_threshold) continue;

      PointT pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = snr;
      radarcloud_xyzi->points.push_back(pt);
    }

    radarcloud_xyzi->width = radarcloud_xyzi->points.size();
    radarcloud_xyzi->height = 1;
    radarcloud_xyzi->is_dense = false;

    //********** Publish PointCloud2 Format Raw Cloud **********
    sensor_msgs::msg::PointCloud2 pc2_raw_msg;
    pcl::toROSMsg(*radarcloud_xyzi, pc2_raw_msg);
    pc2_raw_msg.header.stamp = eagle_msg->header.stamp;
    pc2_raw_msg.header.frame_id = baselinkFrame;
    pc2_raw_pub->publish(pc2_raw_msg);

    // The ego velocity estimator needs a field called "doppler"
    // Pass the original cloud (not the converted one) with the field renamed
    sensor_msgs::msg::PointCloud2 pc2_for_estimator = *eagle_msg;
    pc2_for_estimator.header.frame_id = baselinkFrame;
    for (auto& field : pc2_for_estimator.fields) {
        if (field.name == "radial_velocity") field.name = "doppler";
        else if (field.name == "signal_noise_ratio") field.name = "intensity";
    }
    //********** Ego Velocity Estimation **********
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::msg::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();
    if (estimator.estimate(pc2_for_estimator, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
    {
      clock_t end_ms = clock();
      double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
      egovel_time.push_back(time_used);

      geometry_msgs::msg::TwistWithCovarianceStamped twist;
      twist.header.stamp         = pc2_raw_msg.header.stamp;
      twist.twist.twist.linear.x = v_r.x();
      twist.twist.twist.linear.y = v_r.y();
      twist.twist.twist.linear.z = v_r.z();
      twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
      twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
      twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

      pubtwist->publish(twist);
      pubinlier_pc2->publish(inlier_radar_msg);
      puboutlier_pc2->publish(outlier_radar_msg);
    }

    pcl::PointCloud<PointT>::Ptr radarcloud_inlier(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(inlier_radar_msg, *radarcloud_inlier);

    pcl::PointCloud<PointT>::ConstPtr src_cloud;
    if (enable_dynamic_object_removal)
      src_cloud = radarcloud_inlier;
    else
      src_cloud = radarcloud_xyzi;

    if (src_cloud->empty()) return;

    src_cloud = deskewing(src_cloud);

    if (!baselinkFrame.empty()) {
      try {
        auto transform = tf_buffer->lookupTransform(
          baselinkFrame, src_cloud->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(2.0));
        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
        transformed->header.frame_id = baselinkFrame;
        transformed->header.stamp = src_cloud->header.stamp;
        src_cloud = transformed;
      } catch (const tf2::TransformException& e) {
        RCLCPP_ERROR(get_logger(), "TF failed: %s", e.what());
        return;
      }
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    sensor_msgs::msg::PointCloud2 ros_filtered;
    pcl::toROSMsg(*filtered, ros_filtered);
    pcl_conversions::fromPCL(filtered->header, ros_filtered.header);
    points_pub->publish(ros_filtered);
  }


  pcl::PointCloud<PointT>::ConstPtr passthrough(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    PointT pt;
    for(int i = 0; i < cloud->size(); i++){
      if (cloud->at(i).z < 10 && cloud->at(i).z > -2){
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        pt.intensity = (*cloud)[i].intensity;
        filtered->points.push_back(pt);
      }
    }
    filtered->header = cloud->header;
    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      // Remove NaN/Inf points
      pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
      
      return cloudout;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    filtered->reserve(cloud->size());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      double z = p.z;
      double azimuth_deg = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x)) * 180.0 / M_PI;
      bool in_distance = d > distance_near_thresh && d < distance_far_thresh;
      bool in_height = z < z_high_thresh && z > z_low_thresh;
      bool in_azimuth = !use_azimuth_filter || (azimuth_deg > scan_azimuth_min_deg && azimuth_deg < scan_azimuth_max_deg);
      return in_distance && in_height && in_azimuth;
    });
    // for (size_t i=0; i<cloud->size(); i++){
    //   const PointT p = cloud->points.at(i);
    //   double d = p.getVector3fMap().norm();
    //   double z = p.z;
    //   if (d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh)
    //     filtered->points.push_back(p);
    // }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    rclcpp::Time stamp(pcl_conversions::fromPCL(cloud->header.stamp));
    if(imu_queue.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if(colored_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for(int i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }
      sensor_msgs::msg::PointCloud2 ros_colored;
      pcl::toROSMsg(*colored, ros_colored);
      pcl_conversions::fromPCL(colored->header, ros_colored.header);
      colored_pub->publish(ros_colored);
    }

    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg = imu_queue.front();

    auto loc = imu_queue.begin();
    for(; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);
      if(rclcpp::Time((*loc)->header.stamp) > stamp) {
        break;
      }
    }

    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());
    for(int i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

  bool RadarRaw2PointCloudXYZ(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudxyz)
  {
      pcl::PointXYZ point_xyz;
      for(int i = 0; i < raw->size(); i++)
      {
          point_xyz.x = (*raw)[i].x;
          point_xyz.y = (*raw)[i].y;
          point_xyz.z = (*raw)[i].z;
          cloudxyz->points.push_back(point_xyz);
      }
      return true;
  }
  bool RadarRaw2PointCloudXYZI(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudxyzi)
  {
      pcl::PointXYZI radarpoint_xyzi;
      for(int i = 0; i < raw->size(); i++)
      {
          radarpoint_xyzi.x = (*raw)[i].x;
          radarpoint_xyzi.y = (*raw)[i].y;
          radarpoint_xyzi.z = (*raw)[i].z;
          radarpoint_xyzi.intensity = (*raw)[i].intensity;
          cloudxyzi->points.push_back(radarpoint_xyzi);
      }
      return true;
  }

  void command_callback(const std_msgs::msg::String::ConstSharedPtr str_msg) {
    if (str_msg->data == "time") {
      std::sort(egovel_time.begin(), egovel_time.end());
      double median = egovel_time.at(size_t(egovel_time.size() / 2));
      cout << "Ego velocity time cost (median): " << median << endl;
    }
    else if (str_msg->data == "point_distribution") {
      Eigen::VectorXi data(100);
      for (size_t i = 0; i < num_at_dist_vec.size(); i++){ // N
        Eigen::VectorXi& nad = num_at_dist_vec.at(i);
        for (int j = 0; j< 100; j++){
          data(j) += nad(j);
        }
      }
      data /= num_at_dist_vec.size();
      for (int i=0; i<data.size(); i++){
        cout << data(i) << ", ";
      }
      cout << endl;
    }
  }

private:

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  bool use_distance_filter;
  bool use_azimuth_filter;
  double distance_near_thresh;
  double distance_far_thresh;
  double scan_azimuth_min_deg;
  double scan_azimuth_max_deg;
  double z_low_thresh;
  double z_high_thresh;
  double scan_period;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

  cv::Mat Radar_to_livox; // Transform Radar point cloud to LiDAR Frame
  cv::Mat Thermal_to_RGB,Radar_to_Thermal,RGB_to_livox,livox_to_RGB,Change_Radarframe;
  rio::RadarEgoVelocityEstimator estimator;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pubtwist;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubinlier_pc2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr puboutlier_pc2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_raw_pub;

  bool enable_dynamic_object_removal = false;

  std::mutex odom_queue_mutex;
  std::deque<nav_msgs::msg::Odometry> odom_msgs;
  bool publish_tf;

  std::vector<double> egovel_time;

  std::vector<Eigen::VectorXi> num_at_dist_vec;

  std::string topic_twist;
  std::string topic_inlier_pc2;
  std::string topic_outlier_pc2;

  double power_threshold;
  std::string downsample_method;
  double downsample_resolution;
  std::string outlier_removal_method;
  int statistical_mean_k;
  double statistical_stddev;
  double radius_radius;
  int radius_min_neighbors;
  std::string gt_file_location;
  
};

}  // namespace radar_graph_slam

RCLCPP_COMPONENTS_REGISTER_NODE(radar_graph_slam::PreprocessingComponent)
