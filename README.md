# 🛰️ dynamic\_obstacle\_prediction

Real-time LiDAR-based clustering and obstacle motion prediction using Kalman Filtering, TF transformations, and RViz visualization — designed for safe navigation, planning, and semantic mapping applications.

---

## 📦 Package Overview

This ROS package contains two key nodes:

### 🧠 Obstacle Prediction Node

**File**: `src/dynamic_obstacle_prediction_via_kalman_filter.cpp`

* Tracks 2D obstacle centroids using 4-state Kalman Filters (`x, y, vx, vy`)
* Predicts future positions using configurable `prediction_dt`
* Publishes predicted markers to `/predicted_markers` for RViz
* Automatically deletes unclustered prediction tracks
* **Parameters**:

  * `process_noise`: Kalman filter process covariance
  * `measurement_noise`: measurement covariance
  * `publish_rate`: prediction update frequency (Hz)
  * `prediction_dt`: time delta for future position estimation

---

### 🔎 Point Cloud Clustering Node

**File**: `src/pointcloud_cluster.cpp`

* Performs voxel filtering + Euclidean clustering on raw LiDAR data
* Calculates oriented 3D bounding boxes using PCL moment of inertia estimation
* Transforms clusters from `lidar_frame` to `map` using TF
* Publishes:

  * `/clustered_cloud`: filtered and clustered point cloud
  * `/cluster_markers`: bounding box visualizations in RViz
* **Parameters**:

  * `xy_cluster_tolerance`, `z_cluster_tolerance`: clustering thresholds
  * `leaf_size`: voxel filter resolution
  * `text_size_scale`, `xy_padding_range`: marker display tuning
  * `max_missed_frames`: lifetime tolerance for untracked objects
  * `lidar_frame`: TF frame ID for transformation

---

## 🚀 Launch & TF Setup

**Launch File**: `launch/pointcloud_cluster.launch`

* Launches both `pointcloud_cluster` and `obstacle_predictor_node` under the `veh` namespace
* Includes remapping for:

  * `lidar_points`: input cloud topic
  * `pose_in`: vehicle pose or odometry input
* Uses `tf::TransformListener` for converting LiDAR frame to global map frame

---

## ⚙️ Build Instructions

1. Clone this repo into your ROS workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone <repo_url>
   cd ..
   catkin_make
   ```

2. Source your workspace:

   ```bash
   source devel/setup.bash
   ```

---

## 📄 Dependencies

### ROS Msgs

* `sensor_msgs`
* `visualization_msgs`
* `geometry_msgs`
* `nav_msgs`

### Libraries

* `pcl_ros`
* `tf`
* `Eigen`

Check `package.xml` and `CMakeLists.txt` for full declaration.

---

## 🧩 Utilities

**Header**: `include/box.hpp`

* Defines a custom 2D box structure with position and velocity
* Used in the Kalman tracking process

---

## 📈 Applications

This package enables:

* LiDAR-based dynamic obstacle prediction
* Real-time RViz visualization
* Integration with planning, semantic mapping, and reinforcement learning

Optional:

* Export predicted obstacles as custom ROS messages
* Add predicted velocity vectors in RViz markers

---

🛠️ *Developed as part of a modular perception stack for autonomous surface vehicles and mobile robotics.*
