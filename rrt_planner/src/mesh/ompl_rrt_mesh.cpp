//
// Created by jonasgerstner on 25.02.20.
//
#include "rrt_planner/mesh/ompl_rrt_mesh.h"
#include "rrt_planner/mesh/rrt_setup_mesh.h"

MeshOmplRrt::MeshOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
OmplRrt(nh, nh_private)
{
    problem_setup_ = std::make_shared<MeshRrtSetup>(turning_radius_);

    float dim_x = 5.0f, dim_y = 2.2f, dim_z = 1.5f;
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

    float ratio_trav;
    nh_private_.param("ratio_traversables", ratio_trav, 0.9f);
    bool debug_fcl;
    nh_private_.param("debug_fcl", debug_fcl, false);
    float car_angle_delta;
    nh_private_.param("car_angle_delta_deg", car_angle_delta, 10.0f);

    problem_setup_->setCarConfig(clearance, wheelbase, track);
    problem_setup_->setValConfig(ratio_trav, search_radius_, car_angle_delta, debug_fcl);

    mesh_sub_ = nh_.subscribe("input_mesh", 1, &MeshOmplRrt::mesh_cb, this);
}

void MeshOmplRrt::mesh_cb(const pcl_msgs::PolygonMeshPtr &input_mesh){
    std::dynamic_pointer_cast<MeshRrtSetup>(problem_setup_)->setEnvironmentMesh(input_mesh);
}

/**
 * @param current_height set the current height to speed up the nn search in the traversability cloud
 */
void MeshOmplRrt::setupProblem(float current_height = 0.0f) {
    // cast pointer to derived class to keep interfaces clean
    std::dynamic_pointer_cast<MeshRrtSetup>(problem_setup_)->setFCLCollisionChecking(current_height);

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
                0.25 / (upper_bound_ - lower_bound_).norm() / 2.0;
    }
    problem_setup_->setStateValidityCheckingResolution(validity_checking_resolution);
}

bool MeshOmplRrt::mapsReady() const {
    return (std::dynamic_pointer_cast<MeshRrtSetup>(problem_setup_)->getNumTrianglesEnv() > 0 && problem_setup_->cloudSet());
}
