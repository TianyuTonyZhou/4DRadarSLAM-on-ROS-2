cmake_minimum_required(VERSION 3.14)
project(radar_graph_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-msse4.2 -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geodesy REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(barometer_bmp388 REQUIRED)

find_library(FAST_GICP_LIB fast_gicp 
  PATHS /home/tianyu-tony-zhou/4DRadarSLAM_ws/install/fast_gicp/lib 
  NO_DEFAULT_PATH REQUIRED)

find_library(NDT_OMP_LIB ndt_omp 
  PATHS /home/tianyu-tony-zhou/4DRadarSLAM_ws/install/ndt_omp/lib 
  NO_DEFAULT_PATH REQUIRED)

# --- Message / Service generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloorCoeffs.msg"
  "msg/ScanMatchingStatus.msg"
  "srv/SaveMap.srv"
  "srv/DumpGraph.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Shared AMENT deps list ---
set(AMENT_DEPS
  rclcpp rclcpp_components pcl_ros geodesy nmea_msgs
  sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros
  cv_bridge image_transport
  ndt_omp visualization_msgs
  barometer_bmp388
)

# --- preprocessing component ---
add_library(preprocessing_component SHARED
  apps/preprocessing_nodelet.cpp
  src/radar_ego_velocity_estimator.cpp
)
target_include_directories(preprocessing_component PUBLIC 
  include
  ${ndt_omp_INCLUDE_DIRS}
  ${fast_gicp_INCLUDE_DIRS}
)
target_link_libraries(preprocessing_component
  ${PCL_LIBRARIES} ${OpenCV_LIBS} OpenMP::OpenMP_CXX ${FAST_GICP_LIB})
ament_target_dependencies(preprocessing_component ${AMENT_DEPS})
rosidl_target_interfaces(preprocessing_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(preprocessing_component
  PLUGIN "radar_graph_slam::PreprocessingComponent"
  EXECUTABLE preprocessing_node)

# --- scan_matching_odometry component ---
add_library(scan_matching_odometry_component SHARED
  apps/scan_matching_odometry_nodelet.cpp
  src/radar_graph_slam/registrations.cpp
  src/radar_graph_slam/keyframe.cpp
)
target_include_directories(scan_matching_odometry_component PUBLIC
  include
  ${ndt_omp_INCLUDE_DIRS}
  ${fast_gicp_INCLUDE_DIRS}
)
target_link_libraries(scan_matching_odometry_component 
  ${PCL_LIBRARIES}
  ${NDT_OMP_LIB}
  ${FAST_GICP_LIB}
  g2o_core g2o_stuff g2o_types_slam3d g2o_types_slam3d_addons
  g2o_solver_pcg g2o_solver_csparse g2o_solver_cholmod
)
ament_target_dependencies(scan_matching_odometry_component ${AMENT_DEPS})
rosidl_target_interfaces(scan_matching_odometry_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(scan_matching_odometry_component
  PLUGIN "radar_graph_slam::ScanMatchingOdometryComponent"
  EXECUTABLE scan_matching_odometry_node)

# --- radar_graph_slam component ---
add_library(radar_graph_slam_component SHARED
  apps/radar_graph_slam_nodelet.cpp
  src/radar_graph_slam/graph_slam.cpp
  src/radar_graph_slam/loop_detector.cpp
  src/radar_graph_slam/Scancontext.cpp
  src/radar_graph_slam/keyframe.cpp
  src/radar_graph_slam/map_cloud_generator.cpp
  src/radar_graph_slam/registrations.cpp
  src/radar_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)
target_include_directories(radar_graph_slam_component PUBLIC
  include
  ${GTSAM_INCLUDE_DIR}
  ${ndt_omp_INCLUDE_DIRS}
  ${fast_gicp_INCLUDE_DIRS}
)
target_link_libraries(radar_graph_slam_component
  ${PCL_LIBRARIES} ${OpenCV_LIBS}
  ${NDT_OMP_LIB}
  ${FAST_GICP_LIB}
  g2o_core g2o_stuff g2o_types_slam3d g2o_types_slam3d_addons
  g2o_solver_pcg g2o_solver_csparse g2o_solver_cholmod
  gtsam OpenMP::OpenMP_CXX)
ament_target_dependencies(radar_graph_slam_component ${AMENT_DEPS})
rosidl_target_interfaces(radar_graph_slam_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(radar_graph_slam_component
  PLUGIN "radar_graph_slam::RadarGraphSlamComponent"
  EXECUTABLE radar_graph_slam_node)

# --- Install ---
install(TARGETS
  preprocessing_component
  scan_matching_odometry_component
  radar_graph_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()