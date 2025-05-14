#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <numeric>
#include <functional>
#include <map>
#include <sstream>
#include <iomanip>

class PointCloudClusterNode {
public:
    struct Track {
        int id;
        Eigen::Vector3f center;
        uint8_t r, g, b;
        int missed;
    };

    PointCloudClusterNode() {
        ros::NodeHandle nh, pnh("~");
        // clustering params
        pnh.param("xy_cluster_tolerance", xy_tol_,        0.3);
        pnh.param("z_cluster_tolerance",  z_tol_,         0.3);
        pnh.param("min_cluster_size",     min_size_,      10);
        pnh.param("max_cluster_size",     max_size_,      25000);
        pnh.param("leaf_size",            leaf_size_,     0.05f);
        pnh.param("publish_rate",         pub_rate_,      10.0);
        pnh.param("max_missed_frames",    max_missed_,    5);
        pnh.param("xy_padding_range",     xy_padding_range_, 0.0);
        pnh.param("text_size_scale",      text_size_scale_,   0.2);
        // Frame id for LiDAR (default "lidar_link")
        std::string lidar_frame_default = "lidar_link";
        pnh.param("lidar_frame", lidar_frame_, lidar_frame_default);

        sub_pc_     = nh.subscribe("/pointcloud_in", 1, &PointCloudClusterNode::cloudCb, this);
        pub_markers_= nh.advertise<visualization_msgs::MarkerArray>("/markers_out",1);
        timer_      = nh.createTimer(ros::Duration(1.0/pub_rate_), &PointCloudClusterNode::timerCb, this);

        srand(static_cast<unsigned>(time(nullptr)));
    }

    ~PointCloudClusterNode() {
        visualization_msgs::MarkerArray clear;
        visualization_msgs::Marker m;
        m.action = visualization_msgs::Marker::DELETEALL;
        clear.markers.push_back(m);
        pub_markers_.publish(clear);
        ros::Duration(0.1).sleep();
    }

private:
    ros::Subscriber sub_pc_;
    ros::Publisher  pub_markers_;
    ros::Timer      timer_;
    tf::TransformListener tf_listener_;

    double    xy_tol_, z_tol_, xy_padding_range_, text_size_scale_;  
    std::string lidar_frame_;
    int       min_size_, max_size_, max_missed_;
    float     leaf_size_;
    double    pub_rate_;

    std::vector<Track> tracks_;
    visualization_msgs::MarkerArray last_markers_;
    int    next_id_{0};

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // obtain lidar->map transform
        tf::StampedTransform tf_map_lidar;
        try {
            tf_listener_.lookupTransform("map", lidar_frame_, ros::Time(0), tf_map_lidar);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
            return;
        }

        // convert and downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *raw);
        if (raw->empty()) { incrementMissed(); clearTracksMarkers(); return; }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(raw);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vg.filter(*filtered);

        // scale Z for clustering
        double scale_z = xy_tol_ / z_tol_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scaled(new pcl::PointCloud<pcl::PointXYZ>());
        scaled->points.reserve(filtered->points.size());
        for (auto &p : filtered->points)
            scaled->points.emplace_back(p.x, p.y, static_cast<float>(p.z * scale_z));
        scaled->width = scaled->points.size();
        scaled->height = 1;

        // euclidean clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(scaled);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(xy_tol_);
        ec.setMinClusterSize(min_size_);
        ec.setMaxClusterSize(max_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(scaled);
        ec.extract(clusters);

        // merge nested
        int n = clusters.size();
        std::vector<pcl::PointXYZ> min_pts(n), max_pts(n);
        for (int i = 0; i < n; ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
            for (auto idx : clusters[i].indices)
                tmp->points.push_back(filtered->points[idx]);
            pcl::getMinMax3D(*tmp, min_pts[i], max_pts[i]);
        }
        std::vector<int> parent(n);
        std::iota(parent.begin(), parent.end(), 0);
        std::function<int(int)> find_p = [&](int x){ return parent[x]==x? x: parent[x]=find_p(parent[x]); };
        auto unite = [&](int a,int b){ a=find_p(a); b=find_p(b); if(a!=b) parent[b]=a; };
        for (int i=0; i<n; ++i)
            for (int j=i+1; j<n; ++j) {
                bool ic = (min_pts[j].x>=min_pts[i].x && max_pts[j].x<=max_pts[i].x &&
                           min_pts[j].y>=min_pts[i].y && max_pts[j].y<=max_pts[i].y &&
                           min_pts[j].z>=min_pts[i].z && max_pts[j].z<=max_pts[i].z);
                bool jc = (min_pts[i].x>=min_pts[j].x && max_pts[i].x<=max_pts[j].x &&
                           min_pts[i].y>=min_pts[j].y && max_pts[i].y<=max_pts[j].y &&
                           min_pts[i].z>=min_pts[j].z && max_pts[i].z<=max_pts[j].z);
                if (ic||jc) unite(i,j);
            }
        std::map<int,pcl::PointIndices> merged;
        for (int i=0; i<n; ++i)
            merged[find_p(i)].indices.insert(
                merged[find_p(i)].indices.end(),
                clusters[i].indices.begin(), clusters[i].indices.end());
        clusters.clear();
        for (auto &kv : merged) clusters.push_back(kv.second);

        // centroids
        struct Cand{ pcl::PointIndices idx; Eigen::Vector3f ctr; };
        std::vector<Cand> cands;
        for (auto &ci : clusters) {
            Eigen::Vector3f sum(0,0,0);
            for (auto idx : ci.indices) {
                auto &p = filtered->points[idx];
                sum += Eigen::Vector3f(p.x,p.y,p.z);
            }
            sum /= static_cast<float>(ci.indices.size());
            cands.push_back({ci,sum});
        }

        // match & spawn
        double thr = xy_tol_*1.5;
        std::vector<bool> used(cands.size(),false);
        visualization_msgs::MarkerArray ma;
        for (auto &tr : tracks_) tr.missed++;
        for (size_t i=0; i<cands.size(); ++i) {
            int best=-1; double bestD=thr;
            for (size_t t=0; t<tracks_.size(); ++t) {
                double d = (tracks_[t].center - cands[i].ctr).norm();
                if (d<bestD) { bestD=d; best=t; }
            }
            if (best>=0) {
                tracks_[best].center = cands[i].ctr;
                tracks_[best].missed = 0;
                publishMarker(tracks_[best], filtered, clusters[i], msg->header, ma, tf_map_lidar);
                used[i] = true;
            }
        }
        for (size_t i=0; i<cands.size(); ++i) if (!used[i]) {
            Track tr; tr.id=next_id_++; tr.center=cands[i].ctr; tr.missed=0;
            tr.r=rand()%256; tr.g=rand()%256; tr.b=rand()%256;
            tracks_.push_back(tr);
            publishMarker(tr, filtered, clusters[i], msg->header, ma, tf_map_lidar);
        }
        // prune
        std::vector<int> to_del;
        for (auto it=tracks_.begin(); it!=tracks_.end();) {
            if (it->missed > max_missed_) {
                to_del.push_back(it->id);
                it = tracks_.erase(it);
            } else ++it;
        }
        for (int id : to_del) {
            visualization_msgs::Marker d;
            d.header.frame_id = "map";
            d.header.stamp    = ros::Time::now();
            d.ns              = "cluster"; d.id = id; d.action = d.DELETE;
            ma.markers.push_back(d);
            d.ns = "cluster_labels";
            ma.markers.push_back(d);
        }
        if (tracks_.empty()) next_id_=0;
        last_markers_ = ma;
    }

    void timerCb(const ros::TimerEvent&) {
        pub_markers_.publish(last_markers_);
    }

    void incrementMissed() {
        tracks_.erase(std::remove_if(
            tracks_.begin(), tracks_.end(),
            [&](auto &t){ return ++t.missed > max_missed_; }),
            tracks_.end()
        );
    }

    void clearTracksMarkers() {
        visualization_msgs::MarkerArray clr;
        visualization_msgs::Marker m;
        m.action = m.DELETEALL;
        clr.markers.push_back(m);
        pub_markers_.publish(clr);
    }

    void publishMarker(const Track& tr,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& f,
                       const pcl::PointIndices& ci,
                       const std_msgs::Header& hdr,
                       visualization_msgs::MarkerArray& ma,
                       const tf::StampedTransform& tf_map_lidar)
    {
        // extract oriented bounding box
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (auto idx : ci.indices) {
            auto &p = f->points[idx];
            pcl::PointXYZRGB q;
            q.x=p.x; q.y=p.y; q.z=p.z;
            q.r=tr.r; q.g=tr.g; q.b=tr.b;
            cloud->points.push_back(q);
        }
        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feat;
        feat.setInputCloud(cloud);
        feat.compute();
        pcl::PointXYZRGB mn, mx, ctr;
        Eigen::Matrix3f rot;
        feat.getOBB(mn, mx, ctr, rot);
        double local_yaw = std::atan2(rot(1,0), rot(0,0));

        // transform OBB center & yaw into map frame
        tf::Vector3 t_local(ctr.x, ctr.y, ctr.z);
        tf::Vector3 t_global = tf_map_lidar * t_local;
        double global_yaw = tf::getYaw(tf_map_lidar.getRotation()) + local_yaw;
        geometry_msgs::Quaternion q_global = tf::createQuaternionMsgFromYaw(global_yaw);

        // cube marker
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = hdr.stamp;
        m.ns              = "cluster";
        m.id              = tr.id;
        m.type            = m.CUBE;
        m.action          = m.ADD;
        m.pose.position.x = t_global.x();
        m.pose.position.y = t_global.y();
        m.pose.position.z = t_global.z();
        m.pose.orientation= q_global;
        m.scale.x         = (mx.x - mn.x) + 2*xy_padding_range_;
        m.scale.y         = (mx.y - mn.y) + 2*xy_padding_range_;
        m.scale.z         = (mx.z - mn.z);
        m.color.r = tr.r/255.0f; m.color.g = tr.g/255.0f; m.color.b = tr.b/255.0f; m.color.a = 0.5f;
        m.lifetime = ros::Duration(0);
        ma.markers.push_back(m);

        // text marker
        visualization_msgs::Marker t=m;
        t.ns    = "cluster_labels";
        t.type  = t.TEXT_VIEW_FACING;
        t.pose.position.z += (mx.z - mn.z)/2.0 + 0.1;
        // dynamic text size
        {
            float box_max = std::max(m.scale.x, m.scale.y);
            t.scale.z = std::max(1.0f, box_max) * text_size_scale_;
        }
        t.color.r = t.color.g = t.color.b = 1.0f;
        t.color.a = 1.0f;
        std::ostringstream ss;
        ss << "ID:"<<tr.id<<" ("<<std::fixed<<std::setprecision(2)
           << t_global.x()<<","<<t_global.y()<<")";
        t.text     = ss.str();
        t.lifetime = ros::Duration(0);
        ma.markers.push_back(t);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_cluster_node");
    PointCloudClusterNode node;
    ros::spin();
    return 0;
}
