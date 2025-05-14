#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MarkerToPointCloudNode {
public:
    MarkerToPointCloudNode() {
        ros::NodeHandle nh, pnh("~");
        pnh.param("target_frame", target_frame_, std::string("lidar_link"));
        pnh.param("publish_rate", publish_rate_, 10.0);
        pnh.param("xy_scale", xy_scale_, 0.1);
        pnh.param("z_scale", z_scale_, 0.1);

        sub_markers_ = nh.subscribe("/markers_in", 1, &MarkerToPointCloudNode::markersCb, this);
        pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_out", 1);
        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), &MarkerToPointCloudNode::timerCb, this);
    }

private:
    ros::Subscriber sub_markers_;
    ros::Publisher pub_cloud_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;
    std::string target_frame_;
    double publish_rate_, xy_scale_, z_scale_;
    sensor_msgs::PointCloud2 last_cloud_;
    bool has_cloud_{false};

    void markersCb(const visualization_msgs::MarkerArrayConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        cloud.is_dense = false;
        cloud.header.frame_id = target_frame_;

        if (msg->markers.empty()) return;

        // Pre-allocate: estimate samples per marker
        size_t per_marker = ((size_t)((msg->markers[0].scale.x / xy_scale_) + 1) *
                             (size_t)((msg->markers[0].scale.y / xy_scale_) + 1) * 2) +
                            ((size_t)((msg->markers[0].scale.x / xy_scale_) + 1) *
                             (size_t)((msg->markers[0].scale.z / z_scale_) + 1) * 2) +
                            ((size_t)((msg->markers[0].scale.y / xy_scale_) + 1) *
                             (size_t)((msg->markers[0].scale.z / z_scale_) + 1) * 2);
        cloud.points.reserve(per_marker * msg->markers.size());

        // Single TF lookup per batch
        const std::string src_frame = msg->markers.front().header.frame_id;
        tf::StampedTransform to_target;
        try {
            tf_listener_.lookupTransform(target_frame_, src_frame, ros::Time(0), to_target);
        } catch (tf::TransformException&) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed");
            return;
        }
        tf::Transform base_tf(to_target.getRotation(), to_target.getOrigin());

        for (const auto& m : msg->markers) {
            if (m.action != visualization_msgs::Marker::ADD || m.type != visualization_msgs::Marker::CUBE)
                continue;

            // Pack color and transparency
            uint8_t r = uint8_t(m.color.r * 255);
            uint8_t g = uint8_t(m.color.g * 255);
            uint8_t b = uint8_t(m.color.b * 255);
            uint8_t a = uint8_t(m.color.a * 255);

            // Compose marker transform
            tf::Transform marker_tf;
            marker_tf.setOrigin(tf::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
            marker_tf.setRotation(tf::Quaternion(m.pose.orientation.x, m.pose.orientation.y,
                                              m.pose.orientation.z, m.pose.orientation.w));
            tf::Transform world_tf = base_tf * marker_tf;

            double hx = m.scale.x * 0.5;
            double hy = m.scale.y * 0.5;
            double hz = m.scale.z * 0.5;

            // Lambda to generate and add a point
            auto emit = [&](double x, double y, double z) {
                tf::Vector3 p_local(x, y, z);
                tf::Vector3 p_world = world_tf * p_local;
                pcl::PointXYZRGBA pt;
                pt.x = p_world.x(); pt.y = p_world.y(); pt.z = p_world.z();
                pt.r = r; pt.g = g; pt.b = b; pt.a = a;
                cloud.points.push_back(pt);
            };

            // Sample faces
            for (double xi = -hx; xi <= hx; xi += xy_scale_) {
                for (double yi = -hy; yi <= hy; yi += xy_scale_) {
                    emit(xi, yi, hz);
                    emit(xi, yi, -hz);
                }
            }
            for (double xi = -hx; xi <= hx; xi += xy_scale_) {
                for (double zi = -hz; zi <= hz; zi += z_scale_) {
                    emit(xi, hy, zi);
                    emit(xi, -hy, zi);
                }
            }
            for (double yi = -hy; yi <= hy; yi += xy_scale_) {
                for (double zi = -hz; zi <= hz; zi += z_scale_) {
                    emit(hx, yi, zi);
                    emit(-hx, yi, zi);
                }
            }
        }

        if (cloud.empty()) return;

        cloud.width = cloud.points.size();
        cloud.height = 1;
        pcl::toROSMsg(cloud, last_cloud_);
        last_cloud_.header.stamp = ros::Time::now();
        has_cloud_ = true;
    }

    void timerCb(const ros::TimerEvent&) {
        if (has_cloud_) pub_cloud_.publish(last_cloud_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_to_pointcloud_node");
    MarkerToPointCloudNode node;
    ros::spin();
    return 0;
}