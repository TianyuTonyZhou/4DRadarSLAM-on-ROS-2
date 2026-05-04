// SPDX-License-Identifier: BSD-2-Clause

#include <radar_graph_slam/registrations.hpp>

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_apdgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace radar_graph_slam {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method(rclcpp::Node::SharedPtr node) {
  using PointT = pcl::PointXYZI;

  // select a registration method (ICP, GICP, NDT)
  std::string registration_method = (node->has_parameter("registration_method") ? node->get_parameter("registration_method").get_value<std::string>() : node->declare_parameter<std::string>("registration_method", "NDT_OMP"));
  if(registration_method == "FAST_GICP") {
    std::cout << "registration: FAST_GICP" << std::endl;
    fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(new fast_gicp::FastGICP<PointT, PointT>());
    gicp->setNumThreads((node->has_parameter("reg_num_threads") ? node->get_parameter("reg_num_threads").get_value<int>() : node->declare_parameter<int>("reg_num_threads", 0)));
    gicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
    gicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
    gicp->setMaxCorrespondenceDistance((node->has_parameter("reg_max_correspondence_distance") ? node->get_parameter("reg_max_correspondence_distance").get_value<double>() : node->declare_parameter<double>("reg_max_correspondence_distance", 2.5)));
    gicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
    return gicp;
  }
  else if(registration_method == "FAST_APDGICP") {
    std::cout << "registration: FAST_APDGICP" << std::endl;
    fast_gicp::FastAPDGICP<PointT, PointT>::Ptr apdgicp(new fast_gicp::FastAPDGICP<PointT, PointT>());
    apdgicp->setNumThreads((node->has_parameter("reg_num_threads") ? node->get_parameter("reg_num_threads").get_value<int>() : node->declare_parameter<int>("reg_num_threads", 0)));
    apdgicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
    apdgicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
    apdgicp->setMaxCorrespondenceDistance((node->has_parameter("reg_max_correspondence_distance") ? node->get_parameter("reg_max_correspondence_distance").get_value<double>() : node->declare_parameter<double>("reg_max_correspondence_distance", 2.5)));
    apdgicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
    apdgicp->setDistVar((node->has_parameter("dist_var") ? node->get_parameter("dist_var").get_value<double>() : node->declare_parameter<double>("dist_var", 0.86)));
    apdgicp->setAzimuthVar((node->has_parameter("azimuth_var") ? node->get_parameter("azimuth_var").get_value<double>() : node->declare_parameter<double>("azimuth_var", 0.5)));
    apdgicp->setElevationVar((node->has_parameter("elevation_var") ? node->get_parameter("elevation_var").get_value<double>() : node->declare_parameter<double>("elevation_var", 1.0)));
    return apdgicp;
  }

  if (!node->has_parameter("reg_resolution")) {
    (node->has_parameter("reg_resolution") ? node->get_parameter("reg_resolution").get_value<double>() : node->declare_parameter<double>("reg_resolution", 1.0));
}

#ifdef USE_VGICP_CUDA
  else if(registration_method == "FAST_VGICP_CUDA") {
    std::cout << "registration: FAST_VGICP_CUDA" << std::endl;
    fast_gicp::FastVGICPCuda<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICPCuda<PointT, PointT>());
    vgicp->setResolution(node->get_parameter("reg_resolution").get_value<double>());
    vgicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
    vgicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
    vgicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
    return vgicp;
  }
#endif
  else if(registration_method == "FAST_VGICP") {
    std::cout << "registration: FAST_VGICP" << std::endl;
    fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setNumThreads((node->has_parameter("reg_num_threads") ? node->get_parameter("reg_num_threads").get_value<int>() : node->declare_parameter<int>("reg_num_threads", 0)));
    vgicp->setResolution(node->get_parameter("reg_resolution").get_value<double>());
    vgicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
    vgicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
    vgicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
    return vgicp;
  } else if(registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
    icp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
    icp->setMaxCorrespondenceDistance((node->has_parameter("reg_max_correspondence_distance") ? node->get_parameter("reg_max_correspondence_distance").get_value<double>() : node->declare_parameter<double>("reg_max_correspondence_distance", 2.5)));
    icp->setUseReciprocalCorrespondences((node->has_parameter("reg_use_reciprocal_correspondences") ? node->get_parameter("reg_use_reciprocal_correspondences").get_value<bool>() : node->declare_parameter<bool>("reg_use_reciprocal_correspondences", false)));
    return icp;
  } else if(registration_method.find("GICP") != std::string::npos) {
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
      gicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
      gicp->setUseReciprocalCorrespondences((node->has_parameter("reg_use_reciprocal_correspondences") ? node->get_parameter("reg_use_reciprocal_correspondences").get_value<bool>() : node->declare_parameter<bool>("reg_use_reciprocal_correspondences", false)));
      gicp->setMaxCorrespondenceDistance((node->has_parameter("reg_max_correspondence_distance") ? node->get_parameter("reg_max_correspondence_distance").get_value<double>() : node->declare_parameter<double>("reg_max_correspondence_distance", 2.5)));
      gicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
      gicp->setMaximumOptimizerIterations((node->has_parameter("reg_max_optimizer_iterations") ? node->get_parameter("reg_max_optimizer_iterations").get_value<int>() : node->declare_parameter<int>("reg_max_optimizer_iterations", 20)));
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
      gicp->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
      gicp->setUseReciprocalCorrespondences((node->has_parameter("reg_use_reciprocal_correspondences") ? node->get_parameter("reg_use_reciprocal_correspondences").get_value<bool>() : node->declare_parameter<bool>("reg_use_reciprocal_correspondences", false)));
      gicp->setMaxCorrespondenceDistance((node->has_parameter("reg_max_correspondence_distance") ? node->get_parameter("reg_max_correspondence_distance").get_value<double>() : node->declare_parameter<double>("reg_max_correspondence_distance", 2.5)));
      gicp->setCorrespondenceRandomness((node->has_parameter("reg_correspondence_randomness") ? node->get_parameter("reg_correspondence_randomness").get_value<int>() : node->declare_parameter<int>("reg_correspondence_randomness", 20)));
      gicp->setMaximumOptimizerIterations((node->has_parameter("reg_max_optimizer_iterations") ? node->get_parameter("reg_max_optimizer_iterations").get_value<int>() : node->declare_parameter<int>("reg_max_optimizer_iterations", 20)));
      return gicp;
    }
  } else {
    if(registration_method.find("NDT") == std::string::npos) {
      std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
      std::cerr << "       : use NDT" << std::endl;
    }

    double ndt_resolution = node->get_parameter("reg_resolution").get_value<double>();
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: NDT " << ndt_resolution << std::endl;
      pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
      ndt->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      int num_threads = (node->has_parameter("reg_num_threads") ? node->get_parameter("reg_num_threads").get_value<int>() : node->declare_parameter<int>("reg_num_threads", 0));
      std::string nn_search_method = (node->has_parameter("reg_nn_search_method") ? node->get_parameter("reg_nn_search_method").get_value<std::string>() : node->declare_parameter<std::string>("reg_nn_search_method", "DIRECT7"));
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if(num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon((node->has_parameter("reg_transformation_epsilon") ? node->get_parameter("reg_transformation_epsilon").get_value<double>() : node->declare_parameter<double>("reg_transformation_epsilon", 0.01)));
      ndt->setMaximumIterations((node->has_parameter("reg_maximum_iterations") ? node->get_parameter("reg_maximum_iterations").get_value<int>() : node->declare_parameter<int>("reg_maximum_iterations", 64)));
      ndt->setResolution(ndt_resolution);
      if(nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if(nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
    }
  }

  return nullptr;
}

}  // namespace radar_graph_slam
