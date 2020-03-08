//
// Created by jonasgerstner on 25.02.20.
//
#include "rrt_planner/voxblox/ompl_rrt_voxblox.h"
#include "rrt_planner/voxblox/rrt_setup_voxblox.h"

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
OmplRrt(nh, nh_private)
{
    if (reedsshepp_)
        problem_setup_ = std::make_shared<VoxbloxRrtSetup>(turning_radius_);
    else
        problem_setup_ = std::make_shared<VoxbloxRrtSetup>();
    float dim_x = 4.0f, dim_y = 2.0f, dim_z = 2.0f;
    nh_private_.param("bb_dim_x", dim_x, dim_x);
    nh_private_.param("bb_dim_y", dim_y, dim_y);
    nh_private_.param("bb_dim_z", dim_z, dim_z);

    problem_setup_->setBBDim(dim_x, dim_y, dim_z);
    problem_setup_->setSearchRadius(search_radius_);

    float clearance = 0.1f;
    nh_private_.getParam("clearance", clearance);
    float wheelbase = 2.875f;
    nh_private_.getParam("wheelbase", wheelbase);
    float track = 1.58f;
    nh_private_.getParam("track", track);

    problem_setup_->setCarConfig(clearance, wheelbase, track);

}

void VoxbloxOmplRrt::setTsdfLayer(
        voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
    tsdf_layer_ = tsdf_layer;
    CHECK_NOTNULL(tsdf_layer_);
    voxel_size_ = tsdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setEsdfLayer(
        voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
    esdf_layer_ = esdf_layer;
    CHECK_NOTNULL(esdf_layer_);
    voxel_size_ = esdf_layer_->voxel_size();
}

/**
 * @param current_height set the current height to speed up the nn search in the traversability cloud
 */
void VoxbloxOmplRrt::setupProblem(float current_height = 0.0f) {
    // cast pointer to derived class to keep interfaces clean
    std::dynamic_pointer_cast<VoxbloxRrtSetup>(problem_setup_)->setTsdfVoxbloxCollisionChecking(tsdf_layer_, current_height);

    problem_setup_->setRrtStar();
    problem_setup_->setDefaultObjective();

    if (lower_bound_ != upper_bound_) {
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(0, lower_bound_.x());
        bounds.setLow(1, lower_bound_.y());

        bounds.setHigh(0, upper_bound_.x());
        bounds.setHigh(1, upper_bound_.y());

        // Define start and goal positions.
        problem_setup_->getGeometricComponentStateSpace()
                ->as<ompl::base::ReedsSheppStateSpace>()
                ->setBounds(bounds);
    }

    // This is a fraction of the space extent! Not actual metric units. For
    // mysterious reasons. Thanks OMPL!
    double validity_checking_resolution = 0.01;
    if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
        // If bounds are set, set this to approximately one voxel.
        validity_checking_resolution =
                voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
    }
    problem_setup_->setStateValidityCheckingResolution(validity_checking_resolution);
}
