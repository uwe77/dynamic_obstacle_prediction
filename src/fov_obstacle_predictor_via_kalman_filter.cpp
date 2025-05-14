// src/pose_obstacle_predictor.cpp

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <Eigen/Dense>

// 6-state KF for vehicle pose: [Px, Py, ψ, vx, vy, ω]ᵀ
class VehicleKF {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,6,1> x_;   // state
  Eigen::Matrix<double,6,6> P_, Q_, R_, H_;
  ros::Time               last_stamp_;
  bool                    initialized_ = false;

  VehicleKF(double meas_pos, double meas_yaw, double meas_vel,
            double proc_pos, double proc_yaw, double proc_vel)
  {
    H_.setIdentity();
    Q_.setZero();   Q_(0,0)=proc_pos; Q_(1,1)=proc_pos; Q_(2,2)=proc_yaw;
                    Q_(3,3)=proc_vel; Q_(4,4)=proc_vel; Q_(5,5)=proc_vel;
    R_.setZero();   R_(0,0)=meas_pos; R_(1,1)=meas_pos; R_(2,2)=meas_yaw;
                    R_(3,3)=meas_vel; R_(4,4)=meas_vel; R_(5,5)=meas_vel;
  }

  void init(double px, double py, double yaw, const ros::Time &stamp) {
    x_.setZero();
    x_(0)=px; x_(1)=py; x_(2)=yaw;
    P_.setIdentity(); P_ *= 1e2;
    last_stamp_   = stamp;
    initialized_  = true;
  }

  void predict(double dt) {
    Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Identity();
    A(0,3)=dt; A(1,4)=dt; A(2,5)=dt;
    x_ = A * x_;
    P_ = A*P_*A.transpose() + Q_;
  }

  void update(const Eigen::Matrix<double,6,1> &z) {
    Eigen::Matrix<double,6,1> y = z - H_*x_;
    // wrap yaw difference
    y(2) = angles::shortest_angular_distance(x_(2), z(2));
    Eigen::Matrix<double,6,6> S = H_*P_*H_.transpose() + R_;
    Eigen::Matrix<double,6,6> K = P_*H_.transpose() * S.inverse();
    x_ += K * y;
    // normalize yaw state
    x_(2) = angles::normalize_angle(x_(2));
    P_  = (Eigen::Matrix<double,6,6>::Identity() - K*H_) * P_;
  }
};

class FOVObstaclePredictor {
public:
  FOVObstaclePredictor()
   : nh_(), pnh_("~"),
     kf_(1.0,0.1,1.0, 0.1,0.01,0.1)
  {
    pnh_.param("prediction_dt",  pred_dt_,           0.5);
    pnh_.param("publish_rate",   pub_rate_,          20.0);
    pnh_.param("text_size_scale", text_size_scale_,   0.2);
    pose_sub_    = nh_.subscribe("/pose_in",    10, &FOVObstaclePredictor::poseCB,   this);
    markers_sub_ = nh_.subscribe("/markers_in", 10, &FOVObstaclePredictor::markerCB,this);
    pub_         = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_markers", 1);
    timer_       = nh_.createTimer(
                     ros::Duration(1.0/pub_rate_),
                     &FOVObstaclePredictor::timerCB, this);
  }

private:
  void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    double px = msg->pose.position.x;
    double py = msg->pose.position.y;
    tf::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w
    );
    q.normalize();
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

    ros::Time now = msg->header.stamp;
    if (!kf_.initialized_) {
      kf_.init(px, py, yaw, now);
      last_px_  = px;
      last_py_  = py;
      last_yaw_ = yaw;
      return;
    }

    uint64_t dt_ns = now.toNSec() - kf_.last_stamp_.toNSec();
    double   dt    = double(dt_ns)/1e9;
    if (dt <= 1e-6) return;

    double vx   = (px  - last_px_) / dt;
    double vy   = (py  - last_py_) / dt;
    double vyaw = angles::shortest_angular_distance(last_yaw_, yaw) / dt;

    kf_.predict(dt);
    Eigen::Matrix<double,6,1> z;
    z << px, py, yaw, vx, vy, vyaw;
    kf_.update(z);

    kf_.last_stamp_ = now;
    last_px_        = px;
    last_py_        = py;
    last_yaw_       = yaw;
  }

  void markerCB(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    last_markers_ = *msg;
  }

  void timerCB(const ros::TimerEvent&) {
    if (!kf_.initialized_ || last_markers_.markers.empty())
      return;

    // copy & predict vehicle pose
    auto kf2 = kf_;
    kf2.predict(pred_dt_);

    double Cx    = kf_.x_(0),
           Cy    = kf_.x_(1),
           psi   = kf_.x_(2);
    double Ppx   = kf2.x_(0),
           Ppy   = kf2.x_(1),
           psi_p = kf2.x_(2);

    visualization_msgs::MarkerArray out;
    out.markers.reserve(last_markers_.markers.size());

    for (auto &m : last_markers_.markers) {
      double Ox = m.pose.position.x,
             Oy = m.pose.position.y;

      // (1) world→local @ current psi
      double dx = Ox - Cx, dy = Oy - Cy;
      double lx =  std::cos(psi)*dx - std::sin(psi)*dy;
      double ly =  std::sin(psi)*dx + std::cos(psi)*dy;

      // (2) local→world @ predicted psi_p
      double c = std::cos(psi_p), s = std::sin(psi_p);
      double px = Cx - (Ppx-Cx) + c*lx + s*ly;
      double py = Cy - (Ppy-Cy) - s*lx + c*ly;

      visualization_msgs::Marker pm = m;
      pm.header.frame_id = "map";
      pm.header.stamp    = ros::Time::now();
      pm.pose.position.x = px;
      pm.pose.position.y = py;
      // orientation/z unchanged
      out.markers.push_back(pm);
      // text marker
      visualization_msgs::Marker t=pm;
      t.ns    = "predicted_labels";
      t.type  = t.TEXT_VIEW_FACING;
      t.pose.position.z += 0.1;
      // dynamic text size
      {
          float box_max = std::max(pm.scale.x, pm.scale.y);
          t.scale.z = std::max(1.0f, box_max) * text_size_scale_;
      }
      t.color.r = t.color.g = t.color.b = 1.0f;
      t.color.a = 1.0f;
      std::ostringstream ss;
      ss << "ID:"<<pm.id<<"(pred)";
      t.text     = ss.str();
      t.lifetime = ros::Duration(0);
      out.markers.push_back(t);
    }

    pub_.publish(out);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber pose_sub_, markers_sub_;
  ros::Publisher  pub_;
  ros::Timer      timer_;

  VehicleKF                         kf_;
  visualization_msgs::MarkerArray   last_markers_;
  double                            pred_dt_, pub_rate_, text_size_scale_;
  double                            last_px_, last_py_, last_yaw_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fov_obstacle_predictor_node");
  FOVObstaclePredictor node;
  ros::spin();
  return 0;
}
