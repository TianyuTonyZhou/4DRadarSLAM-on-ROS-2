#pragma once
#ifndef _UTILITY_RADAR_ODOMETRY_H_
#define _UTILITY_RADAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <opencv/cv.h>
// #include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer
{
public:

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useGpsElevation;
    double gpsCovThreshold;
    double poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Sensor Configuration:
    int downsampleRate;
    double lidarMinRange;
    double lidarMaxRange;

    // IMU
    double imuAccNoise;
    double imuGyrNoise;
    double imuAccBiasN;
    double imuGyrBiasN;
    double imuGravity;
    double imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // voxel filter paprams
    double odometrySurfLeafSize;
    double mappingCornerLeafSize;
    double mappingSurfLeafSize ;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    double surroundingkeyframeAddingDistThreshold; 
    double surroundingkeyframeAddingAngleThreshold; 
    double surroundingKeyframeDensity;
    double surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    double loopClosureFrequency;
    int   surroundingKeyframeSize;
    double historyKeyframeSearchRadius;
    double historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    double historyKeyframeFitnessScore;

    // global map visualization radius
    double globalMapVisualizationSearchRadius;
    double globalMapVisualizationPoseDensity;
    double globalMapVisualizationLeafSize;

    rclcpp::Node* node_;

    explicit ParamServer(rclcpp::Node* node)
    {
        node_ = node;
        robot_id        = node->declare_parameter<std::string>("robot_id",         "nuc_handcart");
        pointCloudTopic = node->declare_parameter<std::string>("pointCloudTopic",  "/radar_enhanced_pcl");
        imuTopic        = node->declare_parameter<std::string>("imuTopic",         "/vectornav/imu");
        odomTopic       = node->declare_parameter<std::string>("odomTopic",        "/odom");
        gpsTopic        = node->declare_parameter<std::string>("gpsTopic",         "/ublox/fix");

        lidarFrame      = node->declare_parameter<std::string>("lidarFrame",       "livox");
        baselinkFrame   = node->declare_parameter<std::string>("baselinkFrame",    "base_link");
        odometryFrame   = node->declare_parameter<std::string>("odometryFrame",    "odom_imu");
        mapFrame        = node->declare_parameter<std::string>("mapFrame",         "map");

        useGpsElevation = node->declare_parameter<bool>("useGpsElevation",         false);
        gpsCovThreshold = node->declare_parameter<double>("gpsCovThreshold",       2.0);
        poseCovThreshold= node->declare_parameter<double>("poseCovThreshold",      25.0);

        savePCD          = node->declare_parameter<bool>("savePCD",                false);
        savePCDDirectory = node->declare_parameter<std::string>("savePCDDirectory","/Downloads/LOAM/");

        downsampleRate  = node->declare_parameter<int>("downsampleRate",           1);
        lidarMinRange   = node->declare_parameter<double>("lidarMinRange",         1.0);
        lidarMaxRange   = node->declare_parameter<double>("lidarMaxRange",         1000.0);

        imuAccNoise     = node->declare_parameter<double>("imuAccNoise",           0.01);
        imuGyrNoise     = node->declare_parameter<double>("imuGyrNoise",           0.001);
        imuAccBiasN     = node->declare_parameter<double>("imuAccBiasN",           0.0002);
        imuGyrBiasN     = node->declare_parameter<double>("imuGyrBiasN",           0.00003);
        imuGravity      = node->declare_parameter<double>("imuGravity",            9.80511);
        imuRPYWeight    = node->declare_parameter<double>("imuRPYWeight",          0.01);
        extRotV  = node->declare_parameter<std::vector<double>>("extrinsicRot",   std::vector<double>());
        extRPYV  = node->declare_parameter<std::vector<double>>("extrinsicRPY",   std::vector<double>());
        extTransV= node->declare_parameter<std::vector<double>>("extrinsicTrans", std::vector<double>());

        // These three Eigen lines are unchanged:
        extRot   = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRotV.data(),   3, 3);
        extRPY   = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRPYV.data(),   3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY  = Eigen::Quaterniond(extRPY);

        odometrySurfLeafSize  = node->declare_parameter<double>("odometrySurfLeafSize",  0.2);
        mappingCornerLeafSize = node->declare_parameter<double>("mappingCornerLeafSize",  0.2);
        mappingSurfLeafSize   = node->declare_parameter<double>("mappingSurfLeafSize",    0.2);

        numberOfCores          = node->declare_parameter<int>("numberOfCores",            2);
        mappingProcessInterval = node->declare_parameter<double>("mappingProcessInterval",0.15);

        surroundingkeyframeAddingDistThreshold  = node->declare_parameter<double>("surroundingkeyframeAddingDistThreshold",  1.0);
        surroundingkeyframeAddingAngleThreshold = node->declare_parameter<double>("surroundingkeyframeAddingAngleThreshold",  0.2);
        surroundingKeyframeDensity              = node->declare_parameter<double>("surroundingKeyframeDensity",               1.0);
        surroundingKeyframeSearchRadius         = node->declare_parameter<double>("surroundingKeyframeSearchRadius",          50.0);

        loopClosureEnableFlag       = node->declare_parameter<bool>("loopClosureEnableFlag",            false);
        loopClosureFrequency        = node->declare_parameter<double>("loopClosureFrequency",           1.0);
        surroundingKeyframeSize     = node->declare_parameter<int>("surroundingKeyframeSize",           50);
        historyKeyframeSearchRadius = node->declare_parameter<double>("historyKeyframeSearchRadius",    10.0);
        historyKeyframeSearchTimeDiff = node->declare_parameter<double>("historyKeyframeSearchTimeDiff",30.0);
        historyKeyframeSearchNum    = node->declare_parameter<int>("historyKeyframeSearchNum",          25);
        historyKeyframeFitnessScore = node->declare_parameter<double>("historyKeyframeFitnessScore",    0.3);

        globalMapVisualizationSearchRadius = node->declare_parameter<double>("globalMapVisualizationSearchRadius", 1e3);
        globalMapVisualizationPoseDensity  = node->declare_parameter<double>("globalMapVisualizationPoseDensity",  10.0);
        globalMapVisualizationLeafSize     = node->declare_parameter<double>("globalMapVisualizationLeafSize",     1.0);

        usleep(100);  // keep as-is
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }

};





sensor_msgs::msg::PointCloud2 publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
    pcl::PointCloud<PointType>::Ptr thisCloud,
    rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg->header.stamp).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation(
        thisImuMsg->orientation.x,
        thisImuMsg->orientation.y,
        thisImuMsg->orientation.z,
        thisImuMsg->orientation.w);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    *rosRoll  = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw   = imuYaw;
}


double pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


double pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
