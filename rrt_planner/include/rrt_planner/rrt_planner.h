#ifndef VOXBLOX_RRT_PLANNER_RRT_PLANNER_H
#define VOXBLOX_RRT_PLANNER_RRT_PLANNER_H

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_msgs/conversions.h>
#include <minkindr_conversions/kindr_msg.h>
#include <voxblox_ros/esdf_server.h>

#include <rrt_planner_msgs/PlannerService.h>
#include "rrt_planner/ompl_rrt.h"
#include "rrt_planner/colors.h"

class RrtPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RrtPlanner(const ros::NodeHandle &nh,
               const ros::NodeHandle &nh_private);

    virtual ~RrtPlanner() {}

    virtual bool plannerServiceCallback(rrt_planner_msgs::PlannerServiceRequest& request,
            rrt_planner_msgs::PlannerServiceResponse& response) = 0;

    bool publishPathCallback(std_srvs::EmptyRequest &request,
                             std_srvs::EmptyResponse &response);

    double getMapDistance(const Eigen::Vector3d &position) const;

    bool checkPathForCollisions(const mav_msgs::EigenTrajectoryPointVector &path,
                                double *t) const;

    void pointcloudMessageCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    virtual void setPointCloudPtrOmpl(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr) = 0;

    void convertToNavMsg(std::vector<geometry_msgs::Pose>& waypoints, nav_msgs::Path& path);

protected:
    void inferValidityCheckingResolution(const Eigen::Vector3d &bounding_box);

    bool sanityCheckWorldAndInputs(const Eigen::Vector3d &start_pos,
                                   const Eigen::Vector3d &goal_pos,
                                   const Eigen::Vector3d &bounding_box) const;

    bool checkStartAndGoalFree(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &goal_pos) const;

    void computeMapBounds(Eigen::Vector3d *lower_bound, Eigen::Vector3d *upper_bound);

    static double computePathLength(std::vector<geometry_msgs::Pose> &waypoints);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher path_marker_pub_;
    ros::Publisher polynomial_trajectory_pub_;
    ros::Publisher path_pub_;
    ros::Publisher waypoint_list_pub_;

    ros::Subscriber sub_traversable_pc_;

    ros::ServiceServer planner_srv_;
    ros::ServiceServer path_pub_srv_;

    std::string frame_id_;
    bool visualize_;
    bool do_smoothing_;

    // Number of meters to inflate the bounding box by for straight-line planning.
    double bounding_box_inflation_m_;
    double num_seconds_to_plan_;
    bool simplify_solution_;

    // Bounds on the size of the map.
    Eigen::Vector3d lower_bound_;
    Eigen::Vector3d upper_bound_;

    // Cache the last trajectory for output
    std::vector<geometry_msgs::Pose> last_path_;
    bool last_path_valid_;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_;
};
#endif  // VOXBLOX_RRT_PLANNER_RRT_PLANNER_H
