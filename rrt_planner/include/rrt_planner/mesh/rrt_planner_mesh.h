#ifndef VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H
#define VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_msgs/conversions.h>
#include <minkindr_conversions/kindr_msg.h>
#include <voxblox_ros/esdf_server.h>

#include <rrt_planner_msgs/PlannerService.h>
#include <rrt_planner/rrt_planner.h>
#include "rrt_planner/ompl_rrt.h"
#include "rrt_planner/mesh/ompl_rrt_mesh.h"

class MeshRrtPlanner : public RrtPlanner {
public:
    MeshRrtPlanner(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private);

    virtual ~MeshRrtPlanner() {}

    bool plannerServiceCallback(rrt_planner_msgs::PlannerServiceRequest &request,
                                rrt_planner_msgs::PlannerServiceResponse &response) override;

    double getMapDistance(const Eigen::Vector3d &position) const;

    bool checkPathForCollisions(const mav_msgs::EigenTrajectoryPointVector &path,
                                double *t) const;

    void setPointCloudPtrOmpl(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr) override {rrt_.getProblemSetup()->setCloud(ptr);}

private:
    void inferValidityCheckingResolution(const Eigen::Vector3d &bounding_box);

    bool sanityCheckWorldAndInputs(const Eigen::Vector3d &start_pos,
                                   const Eigen::Vector3d &goal_pos,
                                   const Eigen::Vector3d &bounding_box) const;

    bool checkStartAndGoalFree(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &goal_pos) const;

    // Planners
    MeshOmplRrt rrt_;
};

#endif  // VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H
