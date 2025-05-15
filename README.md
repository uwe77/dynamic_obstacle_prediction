# 🛰️ dynamic\_obstacle\_prediction

Real-time LiDAR-based clustering and obstacle motion prediction using Kalman Filtering, TF transformations, and RViz visualization — designed for safe navigation, planning, and semantic mapping applications.

![Overview](images/all.gif)

---

## 📦 Package Overview

This ROS package includes four main processing nodes:

### 🧠 Dynamic Obstacle Predictor (Global Frame)

**File**: `src/dynamic_obstacle_predictor_via_kalman_filter.cpp`

* Tracks obstacle centroids using 4-state Kalman Filters (`x, y, vx, vy`)
* Predicts future positions using `ros::Time::now()` (no external pose dependency)
* Deletes unseen tracks and avoids stale state propagation
* Publishes predicted obstacle positions to `/markers_out`
* **Parameters**:

  * `process_noise`, `measurement_noise`
  * `publish_rate`, `prediction_dt`

![Dynamic Prediction](images/dynamic.gif)

### 🧭 FOV Obstacle Predictor (Local Frame)

**File**: `src/fov_obstacle_predictor_via_kalman_filter.cpp`

* Uses 6-state Kalman Filter (`x, y, yaw, vx, vy, ω`) to track ego vehicle motion
* Compensates future prediction with ego-motion in local frame
* Converts clustered obstacles from global frame to ego-local predicted frame
* Publishes `/markers_out` with dynamic labeled text markers

![FOV Prediction](images/fov.gif)

### 🔎 Point Cloud Clustering Node

**File**: `src/pointcloud_cluster.cpp`

* Receives raw point cloud from `/pointcloud_in`
* Performs voxel filtering + Euclidean clustering
* Computes oriented bounding boxes (OBBs) using PCL moment of inertia
* Publishes:

  * `/markers_out`: cluster OBB markers
* **Parameters**:

  * `xy_cluster_tolerance`, `z_cluster_tolerance`, `leaf_size`
  * `text_size_scale`, `xy_padding_range`, `max_missed_frames`
  * `lidar_frame`

![Clustering](images/cluster.gif)

### 📐 Marker-to-PointCloud Converter

**File**: `src/marker_to_pointcloud_node.cpp`

* Converts `CUBE`-type markers from `/markers_in` into `sensor_msgs/PointCloud2`
* Uses TF transform lookup to project cubes to global or local frames
* Publishes reconstructed dense point clouds to `/pointcloud_out`
* **Parameters**:

  * `xy_scale`, `z_scale`: sampling resolution per cube
  * `publish_rate`, `target_frame`

---

## 🚀 Launch Files

Each launch file supports parameterized arguments for topic remapping and configuration.

### 🔹 `dynamic_obstacle_predictor.launch`

* Starts global frame predictor (`dynamic_obstacle_predictor_node`)
* Supports args: input/output topics, filter noise, rate, prediction horizon

### 🔹 `fov_obstacle_predictor.launch`

* Starts local frame predictor (`fov_obstacle_predictor_node`)
* Supports args: input pose/topic remaps, TF frame, output markers

### 🔹 `pointcloud_cluster.launch`

* Starts LiDAR clusterer node (`pointcloud_cluster_node`)
* Fully parameterized for clustering thresholds, visualization scale, frame setup

### 🔹 `marker_to_pointcloud.launch`

* Starts marker converter node with configurable topics and frame settings

### 🔹 `launch_all.launch`

* Starts full perception pipeline:

  * `pointcloud_cluster_node`
  * `dynamic_obstacle_predictor_node`
  * `fov_obstacle_predictor_node`
  * Multiple `marker_to_pointcloud_node` instances:

    * `/cluster_markers` → `/cluster_pc`
    * `/predicted_markers` → `/predicted_pc`
    * `/predicted_local_markers` → `/fov_pc`

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

| Node                              | Kalman State             | Input Topic          | Output Topic      |
| --------------------------------- | ------------------------ | -------------------- | ----------------- |
| `dynamic_obstacle_predictor_node` | `[x, y, vx, vy]`         | `/markers_in`        | `/markers_out`    |
| `fov_obstacle_predictor_node`     | `[x, y, yaw, vx, vy, ω]` | `/markers_in` + pose | `/markers_out`    |
| `marker_to_pointcloud_node`       | N/A (cube sampling)      | `/markers_in`        | `/pointcloud_out` |

---

## 🧰 Development Notes

* All ROS topics use standardized I/O naming
* Marker output can be reconstructed into point clouds for downstream processing
* Multi-agent and namespaced deployment supported via launch args
* Local prediction compensates for ego-motion drift using 6-state KF
* Launch files are fully parameterized for flexibility
* Visual representations (`*.gif`) included for clarity

---

🛠️ *Developed as part of a modular perception and prediction stack for autonomous surface vehicles and mobile robotics.*
