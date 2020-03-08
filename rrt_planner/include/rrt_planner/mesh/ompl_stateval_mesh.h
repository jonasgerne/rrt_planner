//
// Created by jonasgerstner on 03.03.20.
//
#ifndef RRT_PLANNER_OMPL_STATEVAL_SURFEL_H
#define RRT_PLANNER_OMPL_STATEVAL_SURFEL_H

#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <rrt_planner/timing.h>

typedef fcl::BVHModel<fcl::OBBf> Model;

class FCLValidityChecker : public ompl::base::StateValidityChecker {
public:
    FCLValidityChecker(const ompl::base::SpaceInformationPtr &space_info,
                       const std::shared_ptr<Model>& env,
                       const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float starting_height,
                       const BBConfig &boundingbox, const CarConfig &carconfig, float radius) :
            ompl::base::StateValidityChecker(space_info),
            env_(env),
            starting_height_(starting_height),
            wheelbase_(carconfig.wheelbase),
            track_(carconfig.track),
            clearance_(carconfig.clearance),
            radius_(radius){
        rob_geom_.reset(new fcl::Boxf(boundingbox.dim_x, boundingbox.dim_y, boundingbox.dim_z));
        tire_offsets_ << -wheelbase_ / 2.0, wheelbase_ / 2.0, wheelbase_ / 2.0, -wheelbase_ / 2.0, track_ / 2.0,
                track_ / 2.0, -track_ / 2.0, -track_ / 2.0;
        identity_ = fcl::Transform3f::Identity();
        if (cloud)
            kdtree.setInputCloud(cloud);
        else
            ROS_ERROR("No input pointcloud set.");
    }

    ~FCLValidityChecker() override = default;

    bool isValid(const ompl::base::State *state) const override {
        if (!si_->satisfiesBounds(state)) {
            return false;
        }
        Timer coll_check("rrt/coll_check_total");
        bool collision = checkCollisionWithRobot(state);
        coll_check.Stop();
        return !collision;
    }

    bool checkTravMultiple(const ompl::base::State *state, Eigen::Vector3f *projection) const {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        Eigen::Vector2d state_vec(se2_state->getX(), se2_state->getY());
        Eigen::Rotation2Dd rot(se2_state->getYaw());
        std::array<pcl::PointXYZRGBNormal, 4> points;
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector2d tmp = state_vec + rot * tire_offsets_.col(i);
            pcl::PointXYZRGBNormal state_as_point;
            state_as_point.x = static_cast<float>(tmp[0]);
            state_as_point.y = static_cast<float>(tmp[1]);
            state_as_point.z = starting_height_;
            if (!checkTravSingle(state_as_point, nullptr))
                return false;
            points[i] = state_as_point;
        }
        Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 4; ++i) {
            center += points[i].getVector3fMap();
        }
        projection->x() = center.x() / 4.0;
        projection->y() = center.y() / 4.0;
        projection->z() = center.z() / 4.0;
        return true;
    }

    bool checkTravSingle(pcl::PointXYZRGBNormal &search_point, Eigen::Vector3f *projection) const {
        //timing::Timer trav("plan/trav_check");
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_dist;

        uint8_t r_val = 255;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_radius_search, point_radius_squared_dist) > 0) {
            // TODO: just return false if point is not traversable, resolves other todo too
            const auto &nearest_point = kdtree.getInputCloud()->points[point_idx_radius_search[0]];

            // check if nearest neighbor of sampled pose is within certain bounds
            Eigen::Vector3f comp_vector(static_cast<float>(2 * radius_), static_cast<float>(2 * radius_), 2000.0);
            Eigen::Vector3f transform = nearest_point.getVector3fMap() - search_point.getVector3fMap();
            if (!(transform.array().abs() < comp_vector.array()).all()) {
                return false;
            }

            // TODO: resolve division by zero (in case surface normal is perpendicular)
            // project sample point to local surface
            auto &n = nearest_point.getNormalVector3fMap();
            auto new_point = search_point.getVector3fMap() +
                             (transform.dot(n) /
                              Eigen::Vector3f::UnitZ().dot(n)) * Eigen::Vector3f::UnitZ();


            pcl::PointXYZRGBNormal projected;
            projected.getVector3fMap() = new_point;
            point_idx_radius_search.clear();
            point_radius_squared_dist.clear();
            if (kdtree.radiusSearch(projected, radius_, point_idx_radius_search,
                                    point_radius_squared_dist) > 0) {
                int vote = 0;
                for (int neighbor : point_idx_radius_search) {
                    if (kdtree.getInputCloud()->points[neighbor].r == r_val) {
                        // non traversable point in vicinity
                        ++vote;
                    }
                }
                if (vote > std::floor(point_idx_radius_search.size() / 2.0))
                    return false;
            } else {
                // no neighbors near sampled point
                return false;
            }
            if (projection) {
                projection->x() = new_point.x();
                projection->y() = new_point.y();
                projection->z() = new_point.z();
            } else {
                search_point.x = new_point.x();
                search_point.y = new_point.y();
                search_point.z = new_point.z();
            }
            return true;
        } else {
            return false;
        }
    }

    bool checkCollisionWithRobot(const ompl::base::State *state) const {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();

        Eigen::Vector3f projection;
        Timer trav("rrt/traversability");
        if (checkTravMultiple(state, &projection)) {
            trav.Stop();
            Timer fcl_coll("rrt/fcl_coll_check");
            projection.z() += clearance_ + rob_geom_->side.z() / 2.0f;
            fcl::CollisionRequestf request;
            fcl::CollisionResultf result;
            auto tf = fcl::Translation3f(projection) *
                      fcl::AngleAxisf(se2_state->getYaw(), Eigen::Vector3f::UnitZ());
            auto coll = fcl::collide(rob_geom_.get(), tf, env_.get(), identity_, request, result);
            fcl_coll.Stop();
            return (coll > 0);
        }
        trav.Stop();
        return true;
    }

protected:
    std::shared_ptr<fcl::Boxf> rob_geom_;
    std::shared_ptr<Model> env_;

    fcl::Transform3f identity_;

    float starting_height_;

    // later std::vector<std::pair<float, float>> tire_pos_;
    double wheelbase_;
    double track_;
    float clearance_;
    float radius_;

    Eigen::Matrix<double, 2, 4> tire_offsets_;

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
};

#endif //RRT_PLANNER_OMPL_STATEVAL_SURFEL_H
