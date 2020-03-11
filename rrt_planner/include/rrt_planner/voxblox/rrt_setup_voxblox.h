//
// Created by jonasgerstner on 24.02.20.
//

#ifndef RRT_PLANNER_RRT_SETUP_VOXBLOX_H
#define RRT_PLANNER_RRT_SETUP_VOXBLOX_H
#include "rrt_planner/setup.h"
#include "ompl_stateval_voxblox.h"

class VoxbloxRrtSetup : public RrtSetup{
public:
    VoxbloxRrtSetup() : RrtSetup(){}
    VoxbloxRrtSetup(float turning_radius) : RrtSetup(turning_radius){}

    void setTsdfVoxbloxCollisionChecking(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer, float current_height) {
        std::shared_ptr<TsdfVoxbloxValidityCheckerOBB> validity_checker(
                new TsdfVoxbloxValidityCheckerOBB(getSpaceInformation(), tsdf_layer, cloud_, current_height, boundingbox_, carconfig_, valconfig_));

        setStateValidityChecker(ompl::base::StateValidityCheckerPtr(validity_checker));
//        si_->setMotionValidator(
//                ompl::base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::TsdfVoxel>(
//                        getSpaceInformation(), validity_checker)));
    }




    /*void setEsdfVoxbloxCollisionChecking(
            double robot_radius, voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
        std::shared_ptr<EsdfVoxbloxValidityChecker> validity_checker(
                new EsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                               esdf_layer));

        setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
        si_->setMotionValidator(
                base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::EsdfVoxel>(
                        getSpaceInformation(), validity_checker)));
    }*/

};
#endif //RRT_PLANNER_RRT_SETUP_VOXBLOX_H
