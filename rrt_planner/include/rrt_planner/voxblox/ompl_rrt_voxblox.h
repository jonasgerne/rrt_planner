//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_OMPL_RRT_VOXBLOX_H
#define RRT_PLANNER_OMPL_RRT_VOXBLOX_H

#include <voxblox_ros/esdf_server.h>
#include <rrt_planner/ompl_rrt.h>

class VoxbloxOmplRrt : public OmplRrt{
public:
    VoxbloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);
    void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);

    void setupProblem(float current_height) override;

    bool mapsReady() const override{return true;}

protected:
    double voxel_size_;

    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
    voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
};
#endif //RRT_PLANNER_OMPL_RRT_VOXBLOX_H
