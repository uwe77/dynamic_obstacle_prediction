# 🛰️ dynamic\_obstacle\_prediction

Real-time LiDAR-based clustering and obstacle motion prediction using Kalman Filtering, TF transformations, and RViz visualization — designed for safe navigation, planning, and semantic mapping applications.

---

## 📦 Package Overview

This ROS package includes three main predictor nodes:

### 🧠 Dynamic Obstacle Predictor (Global Frame)

**File**: `src/dynamic_obstacle_predictor_via_kalman_filter.cpp`

* Tracks obstacle centroids using 4-state Kalman Filters (`x, y, vx, vy`)
* Predicts future positions using `ros::Time::now()` (no external pose dependency)
* Deletes unseen tracks and avoids stale state propagation
* Publishes predicted obstacle positions to `/predicted_markers`
* **Parameters**:

  * `process_noise`, `measurement_noise`
  * `publish_rate`, `prediction_dt`

### 🧭 FOV Obstacle Predictor (Local Frame)

**File**: `src/fov_obstacle_predictor_via_kalman_filter.cpp`

* Uses 6-state Kalman Filter (`x, y, yaw, vx, vy, ω`) to track ego vehicle motion
* Compensates future prediction with ego-motion in local frame
* Converts clustered obstacles from global frame to ego-local predicted frame
* Publishes `/predicted_local_markers` with dynamic labeled text markers

### 🔎 Point Cloud Clustering Node

**File**: `src/pointcloud_cluster.cpp`

* Performs voxel filtering + Euclidean clustering on raw LiDAR point clouds
* Computes oriented bounding boxes (OBBs) using PCL moment of inertia
* Applies TF transform from `lidar_frame` to global map
* Publishes:

  * `/clustered_cloud`, `/cluster_markers`
* **Parameters**:

  * `xy_cluster_tolerance`, `z_cluster_tolerance`, `leaf_size`
  * `text_size_scale`, `xy_padding_range`, `max_missed_frames`
  * `lidar_frame`

---

## 🚀 Launch Files

### 🔹 `dynamic_obstacle_predictor.launch`

* Starts global frame predictor (`dynamic_obstacle_predictor_node`)

### 🔹 `fov_obstacle_predictor.launch`

* Starts local frame predictor (`fov_obstacle_predictor_node`)

### 🔹 `pointcloud_cluster.launch`

* Starts LiDAR clusterer node (`pointcloud_cluster_node`)

### 🔹 `launch_all.launch`

* Starts all nodes together: clustering, global prediction, FOV transformation

All topics are prefixed with `js` for consistency.

---

## ⚙️ Build Instructions

```bash
cd ~/catkin_ws/src
git clone <repo_url>
cd ..
catkin_make
source devel/setup.bash
```

---

## 📄 Dependencies

### ROS Msgs

* `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `nav_msgs`

### Libraries

* `pcl_ros`, `tf`, `Eigen`

### Removed (no longer needed)

* `dynamic_reconfigure`, `gazebo_msgs`, `OpenCV`

---

## 🧩 Utilities

**Header**: `include/box.hpp`

* Defines 2D box structure with velocity and ID
* Used in prediction logic for visual tracking and labeling

---

## 🧠 Prediction Summary

| Node                              | Kalman State             | Input                         | Output Topic               |
| --------------------------------- | ------------------------ | ----------------------------- | -------------------------- |
| `dynamic_obstacle_predictor_node` | `[x, y, vx, vy]`         | Clustered OBBs                | `/predicted_markers`       |
| `fov_obstacle_predictor_node`     | `[x, y, yaw, vx, vy, ω]` | Vehicle pose + Clustered OBBs | `/predicted_local_markers` |

---

## 🧰 Development Notes

* All ROS nodes and markers renamed for modular clarity
* Tracked objects are uniquely labeled in RViz for interpretability
* Lifecycle management ensures clean removal of old/unseen tracks
* Local prediction fully compensates for ego-motion drift using 6-state KF

---

🛠️ *Developed as part of a modular perception and prediction stack for autonomous surface vehicles and mobile robotics.*
