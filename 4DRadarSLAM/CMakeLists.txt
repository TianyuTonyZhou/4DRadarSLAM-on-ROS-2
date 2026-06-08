cmake_minimum_required(VERSION 3.14)
project(radar_graph_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-msse4.2 -O3)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(OpenCV REQUIRED)
find_package(cv_bridge QUIET)
find_package(image_transport QUIET)
find_package(geodesy QUIET)
find_package(g2o QUIET)
find_package(GTSAM QUIET)
find_package(ndt_omp QUIET)
find_package(visualization_msgs QUIET)
find_package(fast_gicp QUIET)
find_package(barometer_bmp388 QUIET)

set(HAVE_FAST_GICP OFF)
if(fast_gicp_FOUND)
  set(HAVE_FAST_GICP ON)
endif()

set(HAVE_NDT_OMP OFF)
if(ndt_omp_FOUND)
  set(HAVE_NDT_OMP ON)
endif()

set(BUILD_RADAR_GRAPH_SLAM_COMPONENT ON)
foreach(optional_dep geodesy g2o GTSAM barometer_bmp388 visualization_msgs cv_bridge image_transport)
  if(NOT ${optional_dep}_FOUND)
    set(BUILD_RADAR_GRAPH_SLAM_COMPONENT OFF)
  endif()
endforeach()

if(NOT BUILD_RADAR_GRAPH_SLAM_COMPONENT)
  message(WARNING
    "Building without radar_graph_slam_component because optional backend "
    "dependencies are missing. This workspace currently lacks one or more of: "
    "geodesy, g2o, GTSAM, barometer_bmp388, visualization_msgs, cv_bridge, image_transport.")
endif()

set(RADAR_GRAPH_SLAM_G2O_LIBS "")
if(g2o_FOUND)
  list(APPEND RADAR_GRAPH_SLAM_G2O_LIBS
    g2o::core
    g2o::stuff
    g2o::solver_pcg
    g2o::solver_cholmod
    g2o::solver_csparse
    g2o::types_slam3d
    g2o::types_slam3d_addons
  )
endif()

add_definitions(${PCL_DEFINITIONS})

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
  rclcpp rclcpp_components message_filters pcl_conversions pcl_ros
  nmea_msgs sensor_msgs geometry_msgs nav_msgs std_msgs
  tf2_ros tf2_eigen tf2_geometry_msgs angles
)

# --- preprocessing component ---
add_library(preprocessing_component SHARED
  apps/preprocessing_nodelet.cpp
  src/radar_ego_velocity_estimator.cpp
)
target_include_directories(preprocessing_component PUBLIC 
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(preprocessing_component
  ${PCL_LIBRARIES} ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(preprocessing_component OpenMP::OpenMP_CXX)
endif()
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
)
target_include_directories(scan_matching_odometry_component PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_matching_odometry_component ${PCL_LIBRARIES})
target_compile_definitions(scan_matching_odometry_component PRIVATE
  $<$<BOOL:${HAVE_FAST_GICP}>:RADAR_GRAPH_SLAM_HAVE_FAST_GICP>
  $<$<BOOL:${HAVE_NDT_OMP}>:RADAR_GRAPH_SLAM_HAVE_NDT_OMP>
)
if(HAVE_NDT_OMP)
  ament_target_dependencies(scan_matching_odometry_component ndt_omp)
endif()
if(HAVE_FAST_GICP)
  ament_target_dependencies(scan_matching_odometry_component fast_gicp)
endif()
ament_target_dependencies(scan_matching_odometry_component ${AMENT_DEPS})
rosidl_target_interfaces(scan_matching_odometry_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(scan_matching_odometry_component
  PLUGIN "radar_graph_slam::ScanMatchingOdometryComponent"
  EXECUTABLE scan_matching_odometry_node)

# --- radar_graph_slam component ---
if(BUILD_RADAR_GRAPH_SLAM_COMPONENT)
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
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(radar_graph_slam_component PRIVATE
    $<$<BOOL:${HAVE_FAST_GICP}>:RADAR_GRAPH_SLAM_HAVE_FAST_GICP>
    $<$<BOOL:${HAVE_NDT_OMP}>:RADAR_GRAPH_SLAM_HAVE_NDT_OMP>
  )
  target_link_libraries(radar_graph_slam_component
    ${PCL_LIBRARIES} ${OpenCV_LIBS}
    gtsam
    ${RADAR_GRAPH_SLAM_G2O_LIBS})
  if(OpenMP_CXX_FOUND)
    target_link_libraries(radar_graph_slam_component OpenMP::OpenMP_CXX)
  endif()
  if(HAVE_NDT_OMP)
    ament_target_dependencies(radar_graph_slam_component ndt_omp)
  endif()
  if(HAVE_FAST_GICP)
    ament_target_dependencies(radar_graph_slam_component fast_gicp)
  endif()
  ament_target_dependencies(radar_graph_slam_component
    ${AMENT_DEPS}
    geodesy
    visualization_msgs
    cv_bridge
    image_transport
    barometer_bmp388
  )
  rosidl_target_interfaces(radar_graph_slam_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rclcpp_components_register_node(radar_graph_slam_component
    PLUGIN "radar_graph_slam::RadarGraphSlamComponent"
    EXECUTABLE radar_graph_slam_node)
endif()

# --- Install ---
set(INSTALL_TARGETS
  preprocessing_component
  scan_matching_odometry_component
)
if(BUILD_RADAR_GRAPH_SLAM_COMPONENT)
  list(APPEND INSTALL_TARGETS radar_graph_slam_component)
endif()

install(TARGETS ${INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  src/radar_graph_slam/map2odom_publisher.py
  src/radar_graph_slam/bag_player.py
  src/radar_graph_slam/ford2bag.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
