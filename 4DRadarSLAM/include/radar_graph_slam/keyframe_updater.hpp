// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace radar_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  explicit KeyframeUpdater(rclcpp::Node::SharedPtr node)
    : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
      if (!node->has_parameter("keyframe_delta_trans"))
          (node->has_parameter("keyframe_delta_trans") ? node->get_parameter("keyframe_delta_trans").get_value<double>() : node->declare_parameter<double>("keyframe_delta_trans", 2.0));
      keyframe_delta_trans = node->get_parameter("keyframe_delta_trans").get_value<double>();
      if (!node->has_parameter("keyframe_delta_angle"))
          (node->has_parameter("keyframe_delta_angle") ? node->get_parameter("keyframe_delta_angle").get_value<double>() : node->declare_parameter<double>("keyframe_delta_angle", 2.0));
      keyframe_delta_angle = node->get_parameter("keyframe_delta_angle").get_value<double>();
      if (!node->has_parameter("keyframe_min_size"))
          (node->has_parameter("keyframe_min_size") ? node->get_parameter("keyframe_min_size").get_value<int>() : node->declare_parameter<int>("keyframe_min_size", 1000));
      keyframe_min_size = node->get_parameter("keyframe_min_size").get_value<int>();
      accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool decide(const Eigen::Isometry3d& pose, const rclcpp::Time& stamp) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      prev_keypose = pose;
      prev_keytime = stamp.seconds();
      return true;
    }
    
    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle();
    // double dt = stamp.toSec() - prev_keytime;

    // too close to the previous frame
    if((dx < keyframe_delta_trans && da < keyframe_delta_angle)) {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    prev_keytime = stamp.seconds();

    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const {
    return accum_distance;
  }

private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //
  std::size_t keyframe_min_size;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
  double prev_keytime;
};

}  // namespace radar_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
