#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>

// 2D Kalman Filter for [x, y, vx, vy]
struct KF2D {
  Eigen::Vector4d x;
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

struct Track {
  KF2D kf;
  visualization_msgs::Marker marker;
};

class ObstaclePredictor {
public:
  ObstaclePredictor() {
    ros::NodeHandle pnh("~");
    pnh.param("measurement_noise", meas_noise_,   1.0);
    pnh.param("process_noise",     proc_noise_,    0.1);
    pnh.param("prediction_dt",     pred_dt_,       0.5);
    pnh.param("publish_rate",      publish_rate_, 10.0);

    markers_sub_ = nh_.subscribe("/markers_in", 10, &ObstaclePredictor::clusterCB, this);
    pred_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_markers", 10);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &ObstaclePredictor::publishTimerCB, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber markers_sub_;
  ros::Publisher pred_pub_;
  ros::Timer publish_timer_;

  double meas_noise_, proc_noise_, pred_dt_, publish_rate_;
  std::unordered_map<int, Track> tracks_;

  void clusterCB(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    ros::Time now = ros::Time::now();

    // Determine which IDs are present this cycle
    std::unordered_set<int> incoming;
    for (const auto& m : msg->markers) {
      if (m.type != visualization_msgs::Marker::CUBE) continue;
      incoming.insert(m.id);
    }

    // Erase tracks no longer seen
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (!incoming.count(it->first)) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }

    // Update or add tracks for current markers
    for (const auto& m : msg->markers) {
      if (m.type != visualization_msgs::Marker::CUBE) continue;
      int id = m.id;
      double mx = m.pose.position.x;
      double my = m.pose.position.y;

      auto it = tracks_.find(id);
      if (it == tracks_.end()) {
        Track tr;
        tr.kf.init(mx, my, meas_noise_, proc_noise_, now);
        tr.marker = m;
        tracks_[id] = tr;
      } else {
        it->second.marker = m;
      }
      Track& tr = tracks_[id];

      double dt = (now - tr.kf.last_stamp).toSec();
      if (dt <= 0) dt = 1e-3;
      tr.kf.predict(dt);
      tr.kf.update(Eigen::Vector2d(mx, my));
      tr.kf.last_stamp = now;
    }
  }

  void publishTimerCB(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray msg;

    // clear previous predicted markers
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = now;
    del.ns = "predicted";
    del.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(del);

    // publish new predictions
    for (auto& kv : tracks_) {
      Track& tr = kv.second;
      KF2D future = tr.kf;
      future.predict(pred_dt_);

      visualization_msgs::Marker pm = tr.marker;
      pm.header.stamp = now;
      pm.ns = "predicted";
      pm.pose.position.x = future.x(0);
      pm.pose.position.y = future.x(1);
      msg.markers.push_back(pm);
    }

    pred_pub_.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_predictor_node");
  ObstaclePredictor node;
  ros::spin();
  return 0;
}
