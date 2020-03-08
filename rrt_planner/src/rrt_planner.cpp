//
// Created by jonasgerstner on 24.02.20.
//
#include <geometry_msgs/PoseArray.h>

#include "rrt_planner/rrt_planner.h"
#include <pcl/common/common.h>


RrtPlanner::RrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    frame_id_("world"),
    visualize_(true){

    waypoint_list_pub_ = nh_private_.advertise<nav_msgs::Path>("waypoint_list", 1);
    path_pub_srv_ = nh_private_.advertiseService("publish_path", &RrtPlanner::publishPathCallback, this);

    sub_traversable_pc_ = nh_private_.subscribe("pointcloud_traversable", 1, &RrtPlanner::pointcloudMessageCallback, this);
}

bool RrtPlanner::publishPathCallback(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
    if (!last_path_valid_){
        ROS_ERROR("Can't publish path, marked as invalid.");
        return false;
    }
    geometry_msgs::PoseArray pose_array;
    for (geometry_msgs::Pose p : last_path_)
        pose_array.poses.push_back(p);

    pose_array.header.frame_id = frame_id_;
    waypoint_list_pub_.publish(pose_array);
    return true;
}

void RrtPlanner::computeMapBounds(Eigen::Vector3d *lower_bound, Eigen::Vector3d *upper_bound) {
    pcl::PointXYZRGBNormal min, max;
    pcl::getMinMax3D(*cloud_, min, max);
    *lower_bound = min.getVector3fMap().cast<double>();
    *upper_bound = max.getVector3fMap().cast<double>();
}

double RrtPlanner::computePathLength(std::vector<geometry_msgs::Pose>& waypoints) {
    double distance = 0.0;
    geometry_msgs::Point last_point;

    for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
        if (it > waypoints.begin())
            distance += sqrt((it->position.x - last_point.x) * (it->position.x - last_point.x)
                               + (it->position.y - last_point.y) * (it->position.y - last_point.y)
                               + (it->position.z - last_point.z) * (it->position.z - last_point.z));
        last_point = it->position;
    }

    return distance;
}

void RrtPlanner::pointcloudMessageCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromROSMsg (*input, *cloud_);
    setPointCloudPtrOmpl(cloud_);
    ROS_INFO("Received traversable points.");
}

void RrtPlanner::convertToNavMsg(std::vector<geometry_msgs::Pose>& waypoints, nav_msgs::Path& path){
    path.header.frame_id = frame_id_;
    for (auto& wp : waypoints){
        geometry_msgs::PoseStamped stamped;
        stamped.pose = wp;
        stamped.header.stamp = ros::Time::now();
        stamped.header.frame_id = frame_id_;
        path.poses.push_back(stamped);
    }
}

