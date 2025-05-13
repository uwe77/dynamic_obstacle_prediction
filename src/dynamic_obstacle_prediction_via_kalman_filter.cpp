#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>

// 2D Kalman Filter for [x, y, vx, vy]
struct KF2D {
  Eigen::Vector4d x;  // state: [x, y, vx, vy]
  Eigen::Matrix4d P, F, Q;
  Eigen::Matrix<double,2,4> H;
  Eigen::Matrix2d R;
  ros::Time last_stamp;

  void init(double mx, double my, double meas_noise, double proc_noise, const ros::Time& t) {
    x << mx, my, 0.0, 0.0;
    P = Eigen::Matrix4d::Identity() * 1e2;
    F = Eigen::Matrix4d::Identity();
    Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << proc_noise, proc_noise, proc_noise, proc_noise;
    H.setZero(); H(0,0)=1; H(1,1)=1;
    R = Eigen::Matrix2d::Identity() * meas_noise;
    last_stamp = t;
  }

  void predict(double dt) {
    F(0,2) = dt;
    F(1,3) = dt;
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(const Eigen::Vector2d& z) {
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    auto K = P * H.transpose() * S.inverse();
    x += K * y;
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
  }
};

// holds filter and last marker for an obstacle
struct Track {
  KF2D kf;
  visualization_msgs::Marker marker;
};

class ObstaclePredictor {
public:
  ObstaclePredictor() {
    ros::NodeHandle pnh("~");
    pnh.param("measurement_noise", meas_noise_, 1.0);
    pnh.param("process_noise",     proc_noise_, 0.1);
    pnh.param("prediction_dt",     pred_dt_,    0.5);
    pnh.param("publish_rate",      publish_rate_,10.0);

    pose_sub_    = nh_.subscribe("/pose_in",        10, &ObstaclePredictor::poseCB,    this);
    cluster_sub_ = nh_.subscribe("/cluster_markers", 10, &ObstaclePredictor::clusterCB, this);
    pred_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_markers", 10);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &ObstaclePredictor::publishTimerCB, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_, cluster_sub_;
  ros::Publisher pred_pub_;
  ros::Timer publish_timer_;

  double meas_noise_, proc_noise_, pred_dt_, publish_rate_;
  geometry_msgs::PoseStamped current_pose_;
  bool have_pose_{false};
  std::unordered_map<int,Track> tracks_;

  void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    have_pose_ = true;
  }

  void clusterCB(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    if (!have_pose_) return;
    ros::Time now = current_pose_.header.stamp;
    std::unordered_set<int> seen;

    // update or init tracks for visible clusters
    for (const auto& m : msg->markers) {
      if (m.type != visualization_msgs::Marker::CUBE) continue;
      int id = m.id;
      seen.insert(id);
      double mx = m.pose.position.x;
      double my = m.pose.position.y;
      // init new track
      if (tracks_.find(id) == tracks_.end()) {
        Track tr;
        tr.kf.init(mx, my, meas_noise_, proc_noise_, now);
        tr.marker = m;
        tracks_[id] = tr;
      }
      // measurement update
      auto &tr = tracks_[id];
      double dt = double(now.toNSec() - tr.kf.last_stamp.toNSec())/1e9;
      if (dt<=0) dt=1e-3;
      tr.kf.predict(dt);
      tr.kf.update(Eigen::Vector2d(mx, my));
      tr.kf.last_stamp = now;
      tr.marker = m;
    }

    // remove tracks for obstacles no longer seen
    std::vector<int> to_remove;
    for (auto &kv : tracks_) {
      if (seen.find(kv.first) == seen.end()) {
        to_remove.push_back(kv.first);
      }
    }
    for (int id : to_remove) {
      tracks_.erase(id);
    }
  }

  void publishTimerCB(const ros::TimerEvent&) {
    if (!have_pose_) return;
    ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray out;
    // clear old
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.ns = "predicted";
    out.markers.push_back(del);

    // publish current predictions
    for (auto &kv : tracks_) {
      auto &tr = kv.second;
      KF2D future = tr.kf;
      future.predict(pred_dt_);
      visualization_msgs::Marker pm = tr.marker;
      pm.header.stamp = now;
      pm.ns = "predicted";
      pm.pose.position.x = future.x(0);
      pm.pose.position.y = future.x(1);
      out.markers.push_back(pm);
    }
    pred_pub_.publish(out);
  }
};

int main(int argc,char** argv){
  ros::init(argc,argv,"obstacle_predictor_node");
  ObstaclePredictor node;
  ros::spin();
  return 0;
}