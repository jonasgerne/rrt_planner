#ifndef VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_
#define VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>


template<typename VoxelType>
class VoxbloxValidityChecker : public ompl::base::StateValidityChecker {
public:
    VoxbloxValidityChecker(const ompl::base::SpaceInformationPtr &space_info,
                           voxblox::Layer<VoxelType> *layer)
            : ompl::base::StateValidityChecker(space_info),
              layer_(layer) {
        CHECK_NOTNULL(layer);
        voxel_size_ = layer->voxel_size();
    }

    bool isValid(const ompl::base::State *state) const override {
        if (!si_->satisfiesBounds(state)) {
            return false;
        }

        voxblox::timing::Timer tsdf_coll("plan/coll_check_total");
        bool collision = checkCollisionWithRobot(state);
        tsdf_coll.Stop();
        return !collision;
    }

    // Returns whether there is a collision: true if yes, false if not.
    virtual bool checkCollisionWithRobot(const ompl::base::State *state) const = 0;

//    virtual bool checkCollisionWithRobotAtVoxel(
//            const voxblox::GlobalIndex &global_index) const {
//        return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
//    }

    float voxel_size() const { return voxel_size_; }

protected:
    voxblox::Layer<VoxelType> *layer_;
    float voxel_size_;
};

class TsdfVoxbloxValidityCheckerOBB : public VoxbloxValidityChecker<voxblox::TsdfVoxel> {
public:
    TsdfVoxbloxValidityCheckerOBB(const ompl::base::SpaceInformationPtr &space_info,
                                  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer,
                                  const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float starting_height,
                                  const BBConfig &boundingbox, const CarConfig &carconfig, float radius)
            : VoxbloxValidityChecker(space_info, tsdf_layer),
              treat_unknown_as_occupied_(false),
              starting_height_(starting_height),
              dim_x(boundingbox.dim_x),
              dim_y(boundingbox.dim_y),
              dim_z(boundingbox.dim_z),
              wheelbase_(carconfig.wheelbase),
              track_(carconfig.track),
              clearance_(carconfig.clearance),
              radius_(radius) {
        if (cloud)
            kdtree.setInputCloud(cloud);
        else
            ROS_ERROR("No input pointcloud set.");
        tire_offsets_ << -wheelbase_ / 2.0, wheelbase_ / 2.0, wheelbase_ / 2.0, -wheelbase_ / 2.0, track_ / 2.0,
                track_ / 2.0, -track_ / 2.0, -track_ / 2.0;
    }

    bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }

    void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
        treat_unknown_as_occupied_ = treat_unknown_as_occupied;
    }

    bool checkTravMultiple(const ompl::base::State *state, voxblox::Point *projection) const {
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

    bool checkTravSingle(pcl::PointXYZRGBNormal &search_point, voxblox::Point *projection) const {
        voxblox::timing::Timer trav("plan/trav_check");
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_dist;

        uint8_t r_val = 255;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_radius_search, point_radius_squared_dist) > 0) {
            const auto &nearest_point = kdtree.getInputCloud()->points[point_idx_radius_search[0]];

            // check if nearest neighbor of sampled pose is within certain bounds
            Eigen::Vector3f comp_vector(static_cast<float>(2 * radius_), static_cast<float>(2 * radius_), 2000.0);
            Eigen::Vector3f transform = nearest_point.getVector3fMap() - search_point.getVector3fMap();
            if (!(transform.array().abs() < comp_vector.array()).all()) {
                trav.Stop();
                return false;
            }

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
                for (int neighbor : point_idx_radius_search) {
                    if (kdtree.getInputCloud()->points[neighbor].r == r_val) {
                        trav.Stop();
                        // non traversable point in vicinity
                        return false;
                    }
                }
            } else {
                trav.Stop();
                // no neighbors near sampled point
                return false;
            }
            if (projection) {
                projection->x() = new_point.x();
                projection->y() = new_point.y();
                projection->z() = new_point.z();
            }
            search_point.x = new_point.x();
            search_point.y = new_point.y();
            search_point.z = new_point.z();
            trav.Stop();
            return true;
        } else {
            trav.Stop();
            return false;
        }
    }

    bool checkCollisionWithRobot(const ompl::base::State *state) const override {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        voxblox::Point projection;

        if (checkTravMultiple(state, &projection)) {
            //if (checkTravSingle(state_as_point, &projection)) {
            projection.z() += clearance_ + dim_z * 0.5;
            voxblox::HierarchicalIndexMap block_voxel_list;
            voxblox::timing::Timer getbb("rrt/get_bb");
            voxblox::utils::getOrientedBoundingBox(*layer_, projection, dim_x, dim_y, dim_z, se2_state->getYaw(),
                                                   &block_voxel_list);
            getbb.Stop();
            voxblox::timing::Timer coll_tsdf("rrt/coll_tsdf");
            for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv : block_voxel_list) {

                // Get block -- only already existing blocks are in the list.
                voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
                        layer_->getBlockPtrByIndex(kv.first);

                if (!block_ptr) {
                    continue;
                }

                for (const voxblox::VoxelIndex &voxel_index : kv.second) {
                    if (!block_ptr->isValidVoxelIndex(voxel_index)) {
                        if (treat_unknown_as_occupied_) {
                            coll_tsdf.Stop();
                            return true;
                        }
                        continue;
                    }
                    const voxblox::TsdfVoxel &tsdf_voxel =
                            block_ptr->getVoxelByVoxelIndex(voxel_index);
                    if (tsdf_voxel.weight < voxblox::kEpsilon) {
                        if (treat_unknown_as_occupied_) {
                            coll_tsdf.Stop();
                            return true;
                        }
                        continue;
                    }
                    if (tsdf_voxel.distance <= 0.0f) {
                        coll_tsdf.Stop();
                        return true;
                    }
                }
            }

            // No collision if nothing in the sphere had a negative or 0 distance.
            // Unknown space is unoccupied, since this is a very optimistic global
            // planner.
            coll_tsdf.Stop();
            return false;
        }
        return true;
    }

protected:
    bool treat_unknown_as_occupied_;

    // initial height of the query to speed up projection of sampled 2D states onto the 3D traversability pointcloud
    float starting_height_;

    // dimensions of the current query's bounding box
    float dim_x;
    float dim_y;
    float dim_z;

    // later std::vector<std::pair<float, float>> tire_pos_;
    double wheelbase_;
    double track_;
    float clearance_;
    float radius_;

    Eigen::Matrix<double, 2, 4> tire_offsets_;

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
};
#endif  // VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_
