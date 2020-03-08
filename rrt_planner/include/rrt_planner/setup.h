//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_SETUP_H
#define RRT_PLANNER_SETUP_H
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <mesh_drive_proc/mesh.h>

#include <Eigen/Core>

#include "rrt_planer_util.h"


/**
* from OMPL: "Create the set of classes typically needed to solve a geometric problem."
* This includes space information, stateValidityCheckers, optimizationObjective etc.
* Setup class for a geometric planning problem with Reeds-Shepp space
*/
class RrtSetup : public ompl::geometric::SimpleSetup {
public:
    RrtSetup(float turning_radius) : ompl::geometric::SimpleSetup(ompl::base::StateSpacePtr(new ompl::base::ReedsSheppStateSpace(turning_radius)))
     {
        setBBDim(0.0f, 0.0f, 0.0f);
     }
    RrtSetup() : ompl::geometric::SimpleSetup(ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace))
    {
        setBBDim(0.0f, 0.0f, 0.0f);
    }

    // Get some defaults.
    void setDefaultObjective() {
        getProblemDefinition()->setOptimizationObjective(
                ompl::base::OptimizationObjectivePtr(
                        new ompl::base::PathLengthOptimizationObjective(
                                getSpaceInformation())));
    }

    void setDefaultPlanner() { setRrtStar(); }

    void setRrtStar() {
        setPlanner(ompl::base::PlannerPtr(
                new ompl::geometric::RRTstar(getSpaceInformation())));
    }

    void setRrtConnect() {
        setPlanner(ompl::base::PlannerPtr(
                new ompl::geometric::RRTConnect(getSpaceInformation())));
    }

    void setInformedRrtStar() {
        setPlanner(ompl::base::PlannerPtr(
                new ompl::geometric::InformedRRTstar(getSpaceInformation())));
    }

    void setPrm() {
        setPlanner(ompl::base::PlannerPtr(
                new ompl::geometric::PRM(getSpaceInformation())));
    }

    const ompl::base::StateSpacePtr& getGeometricComponentStateSpace() const {
        return getStateSpace();
    }

    void setStateValidityCheckingResolution(double resolution) {
        // This is a protected attribute, so need to wrap this function.
        si_->setStateValidityCheckingResolution(resolution);
    }

    void constructPrmRoadmap(double num_seconds_to_construct) {
        ompl::base::PlannerTerminationCondition ptc =
                ompl::base::timedPlannerTerminationCondition(num_seconds_to_construct);

        std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
                ->constructRoadmap(ptc);
    }

    // Uses the path simplifier WITHOUT using B-spline smoothing which leads to
    // a lot of issues for us.
    void reduceVertices() {
        if (pdef_) {
            const ompl::base::PathPtr& p = pdef_->getSolutionPath();
            if (p) {
                ompl::time::point start = ompl::time::now();
                auto& path = static_cast<ompl::geometric::PathGeometric&>(*p);
                std::size_t num_states = path.getStateCount();

                reduceVerticesOfPath(path);
                // simplifyTime_ member of the parent class.
                simplifyTime_ = ompl::time::seconds(ompl::time::now() - start);
                OMPL_INFORM(
                        "MavSetup: Vertex reduction took %f seconds and changed from %d to "
                        "%d states",
                        simplifyTime_, num_states, path.getStateCount());
                return;
            }
        }
        OMPL_WARN("No solution to simplify");
    }

    // Simplification of path without B-splines.
    void reduceVerticesOfPath(ompl::geometric::PathGeometric& path) {
        const double max_time = 0.1;
        ompl::base::PlannerTerminationCondition ptc =
                ompl::base::timedPlannerTerminationCondition(max_time);

        // Now just call near-vertex collapsing and reduceVertices.
        if (path.getStateCount() < 3) {
            return;
        }

        // try a randomized step of connecting vertices
        bool try_more = false;
        if (ptc == false) {
            try_more = psk_->reduceVertices(path);
        }

        // try to collapse close-by vertices
        if (ptc == false) {
            psk_->collapseCloseVertices(path);
        }

        // try to reduce verices some more, if there is any point in doing so
        int times = 0;
        while (try_more && ptc == false && ++times <= 5) {
            try_more = psk_->reduceVertices(path);
        }
    }

    void setBBDim(const float& dim_x, const float& dim_y, const float& dim_z){
        boundingbox_.dim_x = dim_x;
        boundingbox_.dim_y = dim_y;
        boundingbox_.dim_z = dim_z;
    }

    void setCarConfig(float clearance, float wheelbase = 0.0f, float track = 0.0f){
        carconfig_.clearance = clearance;
        carconfig_.track = track;
        carconfig_.wheelbase = wheelbase;
    }

    float getBbDimX() const {
        return boundingbox_.dim_x;
    }

    float getBbDimY() const {
        return boundingbox_.dim_y;
    }

    float getBbDimZ() const {
        return boundingbox_.dim_z;
    }

    void setCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud) {
        ROS_INFO("Setting PCL in setup.");
        cloud_ = cloud;
    }

    bool cloudSet() const{return cloud_ ? true : false;}

    void setSearchRadius(float radius){search_radius_ = radius;}

protected:
    BBConfig boundingbox_;
    CarConfig carconfig_;
    // radius for nearest neighbor search in the point cloud
    float search_radius_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_;
};
#endif //RRT_PLANNER_SETUP_H
