//
// Created by jonasgerstner on 03.03.20.
//
#ifndef RRT_PLANNER_OMPL_STATEVAL_SURFEL_H
#define RRT_PLANNER_OMPL_STATEVAL_SURFEL_H

#include <ros/ros.h>
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <rrt_planner/timing.h>
#include <ompl/base/StateValidityChecker.h>
#include <rrt_planner/rrt_planer_util.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <rrt_planner/trav_check.h>

typedef fcl::BVHModel<fcl::OBBf> Model;

class FCLValidityChecker : public ompl::base::StateValidityChecker {
public:
    FCLValidityChecker(const ompl::base::SpaceInformationPtr &space_info,
                       const std::shared_ptr<Model> &env,
                       const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float starting_height,
                       const BBConfig &boundingbox, const CarConfig &carconfig, const ValConfig &valconfig) :
            ompl::base::StateValidityChecker(space_info),
            env_(env),
            starting_height_(starting_height),
            wheelbase_(carconfig.wheelbase),
            track_(carconfig.track),
            clearance_(carconfig.clearance),
            radius_(valconfig.radius),
            ratio_trav_(valconfig.ratio_trav),
            bb_dim_z_(boundingbox.dim_z),
            debug_fcl_(valconfig.debug_fcl),
            car_angle_delta_(valconfig.car_angle_delta) {
        // set up the collision geometry for the vehicle, the bounding box is centered at zero
        rob_geom_.reset(new fcl::Boxf(boundingbox.dim_x, boundingbox.dim_y, boundingbox.dim_z));
        tire_offsets_ << 0.0, wheelbase_, wheelbase_, 0.0, track_ / 2.0,
                track_ / 2.0, -track_ / 2.0, -track_ / 2.0, 0.0, 0.0, 0.0, 0.0;
        identity_ = fcl::Transform3f::Identity();
        bb_center_ = fcl::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f) * fcl::Translation3f(0.0f, 0.0f, clearance_ + bb_dim_z_ * 0.5f);
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

    template<typename Affine>
    bool checkTravMultiple(const ompl::base::State *state, Affine *transformation) const {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        Eigen::Vector3d state_vec(se2_state->getX(), se2_state->getY(), 0.0);
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(se2_state->getYaw(), Eigen::Vector3d::UnitZ());
        std::array<Eigen::Vector3f, 4> points;
        // Eigen::Matrix4f tmp_proj = Eigen::Matrix4f::Identity();
        Eigen::Vector3f trans(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector3d tmp = state_vec + q * tire_offsets_.col(i);
            pcl::PointXYZRGBNormal state_as_point;
            state_as_point.x = static_cast<float>(tmp[0]);
            state_as_point.y = static_cast<float>(tmp[1]);
            state_as_point.z = starting_height_;
            // tmp_proj.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
            if (!checkTravSingle(state_as_point, &points[i]))
                return false;
            trans += points[i];
        }
        trans /= 4.0f;
        Eigen::Vector3f normal_1 = (points[3] - points[0]).cross(points[1] - points[0]);
        Eigen::Vector3f normal_2 = (points[1] - points[2]).cross(points[3] - points[2]);
        normal_1.normalize();
        normal_2.normalize();
        if (normal_1.dot(normal_2) < car_angle_delta_)
            return false;
        Eigen::Vector3f car_normal = (normal_1 + normal_2) * 0.5f;

        Eigen::Vector3f y_mr = q.matrix().col(1).cast<float>();
        Eigen::Vector3f x_mrp = y_mr.cross(car_normal).normalized();
        Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
        rot.col(0) = x_mrp;
        rot.col(1) = car_normal.cross(x_mrp);
        rot.col(2) = car_normal;
        transformation->translation() = trans;
        transformation->linear() = rot;
        return true;
    }

    /*
    bool checkTravSingle(const pcl::PointXYZRGBNormal &search_point, Eigen::Matrix4f *projection) const {
        //timing::Timer trav("plan/trav_check");
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_dist;

        uint8_t r_val = 255;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_radius_search, point_radius_squared_dist) > 0) {
            const auto &nearest_point = kdtree.getInputCloud()->points[point_idx_radius_search[0]];
            if (nearest_point.r == r_val)
                return false;
            // check if nearest neighbor of sampled pose is within certain bounds
            Eigen::Vector3f transform = nearest_point.getVector3fMap() - search_point.getVector3fMap();
            if (std::abs(nearest_point.x - search_point.x) > radius_ ||
                std::abs(nearest_point.y - search_point.y) > radius_)
                return false;

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
            const auto &n_4f = nearest_point.getNormalVector4fMap();
            projection->coeffRef(0, 3) = new_point.x();
            projection->coeffRef(1, 3) = new_point.y();
            projection->coeffRef(2, 3) = new_point.z();
            Eigen::Vector4f y_mr = projection->col(2);
            Eigen::Vector4f x_mrp = y_mr.cross3(n_4f).normalized();
            projection->col(0) = x_mrp;
            projection->col(1) = n_4f.cross3(x_mrp);
            projection->col(2) = n_4f;
            return true;
        } else {
            return false;
        }
    }
    */
    bool checkTravSingle(const pcl::PointXYZRGBNormal &search_point, Eigen::Vector3f *projection) const {
        //timing::Timer trav("plan/trav_check");
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_dist;

        uint8_t r_val = 255;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_radius_search, point_radius_squared_dist) > 0) {
            const auto &nearest_point = kdtree.getInputCloud()->points[point_idx_radius_search[0]];
            if (nearest_point.r == r_val)
                return false;
            // check if nearest neighbor of sampled pose is within certain bounds
            Eigen::Vector3f transform = nearest_point.getVector3fMap() - search_point.getVector3fMap();
            if (std::abs(nearest_point.x - search_point.x) > radius_ ||
                std::abs(nearest_point.y - search_point.y) > radius_)
                return false;

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
                int non_trav = 0;
                for (int neighbor : point_idx_radius_search) {
                    if (kdtree.getInputCloud()->points[neighbor].r == r_val) {
                        // non traversable point in vicinity
                        ++non_trav;
                    }
                }
                if (1.0f - ((float) non_trav / point_idx_radius_search.size()) < ratio_trav_)
                    return false;
            } else {
                // no neighbors near sampled point
                return false;
            }

            projection->x() = new_point.x();
            projection->y() = new_point.y();
            projection->z() = new_point.z();

            return true;
        } else {
            return false;
        }
    }

    bool checkCollisionWithRobot(const ompl::base::State *state) const {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();

        fcl::Transform3f tf;
        Timer trav("rrt/traversability");
        if (checkTravMultiple<fcl::Transform3f>(state, &tf)) {
            ROS_INFO_STREAM_ONCE(tf.matrix());
            tf = tf * bb_center_;
            trav.Stop();
            Timer fcl_coll("rrt/fcl_coll_check");
            fcl::CollisionRequestf request;
            fcl::CollisionResultf result;
            auto coll = fcl::collide(rob_geom_.get(), tf, env_.get(), identity_, request, result);
            fcl_coll.Stop();
            ROS_INFO_STREAM_COND(debug_fcl_, tf.matrix());
            return (coll > 0);
        }
        trav.Stop();
        return true;
    }

protected:
    std::shared_ptr<fcl::Boxf> rob_geom_;
    std::shared_ptr<Model> env_;

    fcl::Transform3f identity_;
    fcl::Transform3f bb_center_;
    float starting_height_;

    double wheelbase_;
    double track_;
    float clearance_;
    float radius_;
    float bb_dim_z_;
    float car_angle_delta_;

    bool debug_fcl_;

    float ratio_trav_;

    Eigen::Matrix<double, 3, 4> tire_offsets_;

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
};

#endif //RRT_PLANNER_OMPL_STATEVAL_SURFEL_H
