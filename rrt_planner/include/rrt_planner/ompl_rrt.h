//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_OMPL_RRT_H
#define RRT_PLANNER_OMPL_RRT_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "setup.h"
/**
 * Base class for OMPL
 */
class OmplRrt {
public:
    enum RrtPlannerType {
        kRrt = 0,
        kRrtConnect,
        kRrtStar,
        kInformedRrtStar,
        kBitStar,
        kPrm
    };

    OmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~OmplRrt() {}

    virtual void setupProblem(float current_height) = 0;

    void setBounds(const Eigen::Vector3d& lower_bound,
                   const Eigen::Vector3d& upper_bound);

    bool getPathBetweenWaypoints(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, std::vector<geometry_msgs::Pose>& solution, double& reeds_shepp_length);
    bool getPathBetweenWaypoints(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, std::vector<geometry_msgs::Pose>& solution, double& reeds_shepp_length, double seconds_to_plan);

    void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path, std::vector<geometry_msgs::Pose>* trajectory_points) const;

    inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }

    const std::shared_ptr<RrtSetup> &getProblemSetup() const;

    static void poseToState(const geometry_msgs::Pose&, ompl::base::State*);
    void stateToPose(const ompl::base::State*, geometry_msgs::Pose&) const;

    virtual bool mapsReady() const = 0;

protected:
    void setupFromStartAndGoal(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    Eigen::Vector3d lower_bound_;
    Eigen::Vector3d upper_bound_;

    std::shared_ptr<RrtSetup> problem_setup_;

    RrtPlannerType planner_type_;
    double num_seconds_to_plan_;
    bool simplify_solution_;
    bool verbose_;
    bool optimistic_;
    bool reedsshepp_;

    float turning_radius_;
    float search_radius_;

    double distance_between_poses_;
    double state_validity_res_;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_;
};
#endif //RRT_PLANNER_OMPL_RRT_H
