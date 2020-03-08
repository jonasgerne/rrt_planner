//
// Created by jonasgerstner on 24.02.20.
//
#include <geometry_msgs/PoseArray.h>

#include "rrt_planner/voxblox/rrt_planner_voxblox.h"
#include <pcl/common/common.h>

VoxbloxRrtPlanner::VoxbloxRrtPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
        RrtPlanner(nh, nh_private),
        voxblox_server_(nh_, nh_private_),
        rrt_(nh_, nh_private_) {

    planner_srv_ = nh_private_.advertiseService(
            "plan", &VoxbloxRrtPlanner::plannerServiceCallback, this);

    nh_private_.param("voxblox_path", input_filepath_, input_filepath_);


    loadTsdfLayer();
    loadVoxbloxMap();

    rrt_.setOptimistic(true);

    // todo: merge tsdf_layer_ into voxblox_server after loading or read from ROS message
    rrt_.setTsdfLayer(tsdf_layer_.get());

    if (visualize_) {
        voxblox_server_.generateMeshWithPCL();
    }


}

bool VoxbloxRrtPlanner::loadTsdfLayer() {
    if (!input_filepath_.empty()) {
        if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_, &tsdf_layer_)) {
            ROS_FATAL_STREAM("Unable to load a TSDF grid from: " << input_filepath_);
            return false;
        }
        return true;
    }
    return false;
}

bool VoxbloxRrtPlanner::loadVoxbloxMap() {
    if (!input_filepath_.empty()) {
        // Verify that the map has an ESDF layer, otherwise generate it.
        if (!voxblox_server_.loadMap(input_filepath_)) {
            ROS_ERROR("Couldn't load ESDF map!");

            // Check if the TSDF layer is non-empty...
            if (tsdf_layer_->getNumberOfAllocatedBlocks() > 0) {
                ROS_INFO("Generating ESDF layer from TSDF.");
                // If so, generate the ESDF layer!

                const bool full_euclidean_distance = true;
                voxblox_server_.updateEsdfBatch(full_euclidean_distance);
            } else {
                ROS_ERROR("TSDF map also empty! Check voxel size!");
            }
        }
    }
}

bool VoxbloxRrtPlanner::plannerServiceCallback(
        rrt_planner_msgs::PlannerServiceRequest &request,
        rrt_planner_msgs::PlannerServiceResponse &response) {

    const auto &start_pose = request.start_pose.pose;
    const auto &goal_pose = request.goal_pose.pose;

    // Setup latest copy of map.
    /*if (!(esdf_map_ &&
          esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) &&
        !(tsdf_map_ &&

          tsdf_layer_->getNumberOfAllocatedBlocks() > 0) {
        ROS_ERROR("Both maps are empty!");
        return false;
    }*/
    if (tsdf_layer_->getNumberOfAllocatedBlocks() == 0) {
        ROS_ERROR("Map is empty!");
        return false;
    }

    // Figure out map bounds!
    //computeMapBounds(&lower_bound_, &upper_bound_);
    // DEBUG: decrease bounds to small area
    lower_bound_.x() = -204.0;
    lower_bound_.y() = 196.103;
    upper_bound_.x() = -195.95;
    upper_bound_.y() = 228.0;

    ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                   << upper_bound_.transpose() << " size: "
                                   << (upper_bound_ - lower_bound_).transpose());

    // Inflate the bounds a bit.
    constexpr double kBoundInflationMeters = 0.5;
    // Don't in flate in z. ;)
    rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                  kBoundInflationMeters, 0.0),
                   upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                  kBoundInflationMeters, 0.0));
    rrt_.setupProblem(start_pose.position.z);

    ROS_INFO("Planning path.");

    // TODO: check start and goal on collisions
//    if (getMapDistance(start_pose) < constraints_.robot_radius) {
//        ROS_ERROR("Start pose occupied!");
//        return false;
//    }
//    if (getMapDistance(goal_pose) < constraints_.robot_radius) {
//        ROS_ERROR("Goal pose occupied!");
//        return false;
//    }

    std::vector<geometry_msgs::Pose> waypoints;
    voxblox::timing::Timer rrtstar_timer("plan/rrt_star");

    bool success = rrt_.getPathBetweenWaypoints(start_pose, goal_pose, waypoints, request.max_planner_time);
    rrtstar_timer.Stop();
    double path_length = computePathLength(waypoints);
    int num_vertices = waypoints.size();
    ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (!success) {
        ROS_INFO_STREAM("All timings: "
                                << std::endl
                                << voxblox::timing::Timing::Print());
        return false;
    }

   /* visualization_msgs::MarkerArray marker_array;
    if (visualize_) {
        marker_array.markers.push_back(createMarkerForPath(
                waypoints, frame_id_, vis_colors::Color::Orange(), "rrt_star",
                0.075));
        marker_array.markers.push_back(createMarkerForWaypoints(
                waypoints, frame_id_, vis_colors::Color::Green(),
                "rrt_star_waypoints", 0.15));
    }*/

    nav_msgs::Path path;
    convertToNavMsg(waypoints, path);
    waypoint_list_pub_.publish(path);

    last_path_ = waypoints;

    last_path_valid_ = true;

    response.success = success;
    response.path = path;
    response.path_length = static_cast<float>(path_length);

    ROS_INFO_STREAM("All timings: "
                            << std::endl
                            << voxblox::timing::Timing::Print());
    ROS_INFO_STREAM("Finished planning with start point: "
                            << start_pose.position
                            << " and goal point: " << goal_pose.position);
    return success;
}

