# Testing Guide

This workspace contains ROS 2 packages for radar SLAM and supporting libraries.

Packages in this workspace:

- `radar_graph_slam` in `src/4DRadarSLAM`
- `barometer_bmp388` in `src/barometer_bmp388`
- `fast_gicp` in `src/fast_apdgicp`

## Current Test Status

Automated test coverage is limited.

- `fast_gicp` contains one existing GoogleTest source:
  [gicp_test.cpp](/home/localadmin/ri_slam_ws/src/fast_apdgicp/src/test/gicp_test.cpp)
- `radar_graph_slam` currently has no registered unit or integration tests in this workspace
- `barometer_bmp388` currently has no registered unit or integration tests in this workspace

The `fast_gicp` test target is disabled by default and is only built when `BUILD_test=ON`.

## Prerequisites

Before running tests:

```bash
cd ~/ri_slam_ws
source /opt/ros/<your_distro>/setup.bash
```

If the workspace has already been built and you want package overlays:

```bash
source ~/ri_slam_ws/install/setup.bash
```

## 1. Build Verification

The baseline sanity check for this workspace is a clean package build.

Build all packages:

```bash
cd ~/ri_slam_ws
colcon build --symlink-install
```

Build a single package:

```bash
colcon build --packages-select radar_graph_slam --symlink-install
colcon build --packages-select barometer_bmp388 --symlink-install
colcon build --packages-select fast_gicp --symlink-install
```

What this verifies:

- CMake configuration succeeds
- ROS package manifests resolve
- libraries and executables link
- installed launch files and runtime assets are generated

## 2. Automated Tests

### fast_gicp

The only existing automated test source in this workspace is in `fast_gicp`.

To build the test target:

```bash
cd ~/ri_slam_ws
colcon build \
  --packages-select fast_gicp \
  --cmake-args -DBUILD_test=ON
```

If you also want CUDA-backed variants:

```bash
colcon build \
  --packages-select fast_gicp \
  --cmake-args -DBUILD_test=ON -DBUILD_VGICP_CUDA=ON
```

Run via `ctest`:

```bash
cd ~/ri_slam_ws/build/fast_gicp
ctest --output-on-failure
```

Run the binary directly:

```bash
~/ri_slam_ws/build/fast_gicp/gicp_test /path/to/test_data
```

Required dataset files:

- `relative.txt`
- `251370668.pcd`
- `251371071.pcd`

What this test verifies:

- dataset load
- registration convergence
- forward and backward alignment
- source/target swap behavior
- translation and rotation error tolerance

### radar_graph_slam

There are currently no registered automated tests for `radar_graph_slam`.

Recommended minimum regression check after code changes:

```bash
cd ~/ri_slam_ws
colcon build --packages-select radar_graph_slam --symlink-install
python3 -m py_compile src/4DRadarSLAM/launch/radar_graph_slam.launch.py
```

### barometer_bmp388

There are currently no registered automated tests for `barometer_bmp388`.

Recommended minimum regression check after code changes:

```bash
cd ~/ri_slam_ws
colcon build --packages-select barometer_bmp388 --symlink-install
```

## 3. Manual Runtime Validation

For `radar_graph_slam`, the meaningful verification path is runtime validation with a rosbag.

### Package Discovery

Check that ROS sees the workspace packages:

```bash
cd ~/ri_slam_ws
source /opt/ros/<your_distro>/setup.bash
source install/setup.bash
ros2 pkg list | rg '^(radar_graph_slam|barometer_bmp388|fast_gicp)$'
```

Check available executables:

```bash
ros2 pkg executables radar_graph_slam
```

Expected executables include:

- `preprocessing_node`
- `scan_matching_odometry_node`
- `radar_graph_slam_node`

### Launch Validation

Basic launch smoke test:

```bash
cd ~/ri_slam_ws
source /opt/ros/<your_distro>/setup.bash
source install/setup.bash
ROS_LOG_DIR=~/ri_slam_ws/.ros_log \
ros2 launch radar_graph_slam radar_graph_slam.launch.py \
  bag_path:="/path/to/bag" \
  enable_gps:=false \
  use_rviz:=false
```

Why `ROS_LOG_DIR` is set:

- in restricted environments, ROS may fail if it tries to write under `~/.ros/log`

### Recommended Runtime Checks

During a manual run, inspect for:

- repeated `Too large transform!! ... Ignore this frame`
- repeated `scan matching has not converged`
- repeated `Detected jump back in time. Clearing TF buffer.`
- unexpected node crashes
- RViz freezes or severe trajectory snapping

Useful checks:

```bash
rg -n "Too large transform|has not converged|Detected jump back in time|WARN|ERROR" /home/localadmin/ri_slam_ws/last_radar_run.log
```

## 4. Current Known Good Runtime Defaults

The current launch defaults were adjusted to reduce instability for bag-based radar-only runs:

- `enable_gps := false` for no-GPS evaluation
- `enable_imu_fusion := false`
- `enable_gravity_constraint := false`
- `enable_imu_orientation := false`
- `enable_dynamic_object_removal := true`
- `registration_method := FAST_APDGICP`
- preprocessing uses stronger filtering than the original defaults

These defaults are defined in:

- [radar_graph_slam.launch.py](~/ri_slam_ws/src/4DRadarSLAM/launch/radar_graph_slam.launch.py)

## 5. Suggested Regression Workflow

For code changes in `radar_graph_slam`:

1. Build the package.
2. Validate launch file syntax.
3. Run a short bag playback window without RViz.
4. Inspect warnings and convergence logs.
5. Run a full visual validation with RViz if the short run is clean.

Example:

```bash
cd ~/ri_slam_ws
source /opt/ros/<your_distro>/setup.bash
colcon build --packages-select radar_graph_slam --symlink-install
python3 -m py_compile src/4DRadarSLAM/launch/radar_graph_slam.launch.py
ROS_LOG_DIR=~/ri_slam_ws/.ros_log \
timeout 40s ros2 launch radar_graph_slam radar_graph_slam.launch.py \
  bag_path:="/path/to/bag" \
  enable_gps:=false \
  use_rviz:=false \
  > ~/ri_slam_ws/last_radar_run.log 2>&1
rg -n "Too large transform|has not converged|Detected jump back in time|WARN|ERROR" /home/localadmin/ri_slam_ws/last_radar_run.log
```

For code changes in `fast_gicp`:

1. Build the package.
2. Build with `-DBUILD_test=ON`.
3. Run `ctest --output-on-failure`.
4. If registration code changed, rerun the bag-based SLAM check because `radar_graph_slam` depends on this package.
