// SPDX-License-Identifier: BSD-2-Clause

#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace radar_graph_slam {

class InformationMatrixCalculator {
public:
  using PointT = pcl::PointXYZI;

  InformationMatrixCalculator() {}
  explicit InformationMatrixCalculator(rclcpp::Node::SharedPtr node);
  ~InformationMatrixCalculator();

  void load(rclcpp::Node::SharedPtr node) {
    use_const_inf_matrix = (node->has_parameter("use_const_inf_matrix") ? node->get_parameter("use_const_inf_matrix").get_value<bool>() : node->declare_parameter<bool>("use_const_inf_matrix", false));
    const_stddev_x       = (node->has_parameter("const_stddev_x") ? node->get_parameter("const_stddev_x").get_value<double>() : node->declare_parameter<double>("const_stddev_x", 0.5));
    const_stddev_q       = (node->has_parameter("const_stddev_q") ? node->get_parameter("const_stddev_q").get_value<double>() : node->declare_parameter<double>("const_stddev_q", 0.1));
    
    var_gain_a           = (node->has_parameter("var_gain_a") ? node->get_parameter("var_gain_a").get_value<double>() : node->declare_parameter<double>("var_gain_a", 20.0));
    min_stddev_x         = (node->has_parameter("min_stddev_x") ? node->get_parameter("min_stddev_x").get_value<double>() : node->declare_parameter<double>("min_stddev_x", 0.1));
    max_stddev_x         = (node->has_parameter("max_stddev_x") ? node->get_parameter("max_stddev_x").get_value<double>() : node->declare_parameter<double>("max_stddev_x", 5.0));
    min_stddev_q         = (node->has_parameter("min_stddev_q") ? node->get_parameter("min_stddev_q").get_value<double>() : node->declare_parameter<double>("min_stddev_q", 0.05));
    max_stddev_q         = (node->has_parameter("max_stddev_q") ? node->get_parameter("max_stddev_q").get_value<double>() : node->declare_parameter<double>("max_stddev_q", 0.2));
    fitness_score_thresh = (node->has_parameter("fitness_score_thresh") ? node->get_parameter("fitness_score_thresh").get_value<double>() : node->declare_parameter<double>("fitness_score_thresh", 2.5));
  }

  static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max());

  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const;

private:
  double weight(double a, double max_x, double min_y, double max_y, double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
};

}  // namespace radar_graph_slam

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
