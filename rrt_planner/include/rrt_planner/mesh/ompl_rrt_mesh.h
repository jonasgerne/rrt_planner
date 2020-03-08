//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_OMPL_RRT_VOXBLOX_H
#define RRT_PLANNER_OMPL_RRT_VOXBLOX_H

#include <voxblox_ros/esdf_server.h>
#include <rrt_planner/ompl_rrt.h>

class MeshOmplRrt : public OmplRrt{
public:
    MeshOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    void setupProblem(float current_height) override;

    bool mapsReady() const override;

    void mesh_cb(const pcl_msgs::PolygonMeshPtr &input_mesh);
protected:
    ros::Subscriber mesh_sub_;
};
#endif //RRT_PLANNER_OMPL_RRT_VOXBLOX_H
