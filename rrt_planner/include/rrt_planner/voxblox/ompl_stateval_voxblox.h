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
                                  const BBConfig &boundingbox, const CarConfig &carconfig, const ValConfig &valconfig)
            : VoxbloxValidityChecker(space_info, tsdf_layer),
              treat_unknown_as_occupied_(false),
              starting_height_(starting_height),
              bb_dim_x_(boundingbox.dim_x),
              bb_dim_y_(boundingbox.dim_y),
              bb_dim_z_(boundingbox.dim_z),
              wheelbase_(carconfig.wheelbase),
              track_(carconfig.track),
              clearance_(carconfig.clearance),
              radius_(valconfig.radius),
              ratio_trav_(valconfig.ratio_trav),
              car_angle_delta_(valconfig.car_angle_delta){
        if (cloud)
            kdtree.setInputCloud(cloud);
        else
            ROS_ERROR("No input pointcloud set.");
        tire_offsets_ << -wheelbase_ / 2.0, wheelbase_ / 2.0, wheelbase_ / 2.0, -wheelbase_ / 2.0, track_ / 2.0,
                track_ / 2.0, -track_ / 2.0, -track_ / 2.0, 0.0, 0.0, 0.0, 0.0;
        bb_center_ = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f) * Eigen::Translation3f(0.0f, 0.0f, clearance_ + bb_dim_z_ * 0.5f);
    }

    bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }

    void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
        treat_unknown_as_occupied_ = treat_unknown_as_occupied;
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

    bool checkTravSingle(const pcl::PointXYZRGBNormal &search_point, Eigen::Vector3f *projection) const {
        //timing::Timer trav("plan/trav_check");
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_dist;

        uint8_t r_val = 255;

        int k = relax_neighbor_search_ ? 5 : 1;

        if (kdtree.nearestKSearch(search_point, k, point_idx_radius_search, point_radius_squared_dist) > 0) {
            pcl::PointXYZRGBNormal nearest_point;
            for (int i = 0; i < k; ++i) {
                nearest_point = kdtree.getInputCloud()->points[point_idx_radius_search[i]];
                if (nearest_point.g == r_val)
                    break;
                else if (nearest_point.r == r_val && i < k - 1) {
                    continue;}
                else
                    return false;
            }
            // check if nearest neighbor of sampled pose is within certain bounds
            Eigen::Vector3f transform = nearest_point.getVector3fMap() - search_point.getVector3fMap();
            if (std::abs(nearest_point.x - search_point.x) > radius_ ||
                std::abs(nearest_point.y - search_point.y) > radius_)
                return false;

            // project sample point to local surface
            const auto &n = nearest_point.getNormalVector3fMap();
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
                if (1.0f - ((float)non_trav / point_idx_radius_search.size()) < ratio_trav_)
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

    /*
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
    */
    bool checkCollisionWithRobot(const ompl::base::State *state) const override {
        const auto &se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        // voxblox::Point projection;
        Eigen::Transform<float, 3, Eigen::Affine> projection;

        if (checkTravMultiple(state, &projection)) {
            ROS_INFO_STREAM_ONCE(projection.matrix());
            projection = projection * bb_center_;
            ROS_INFO_STREAM_ONCE(projection.matrix());
            voxblox::HierarchicalIndexMap block_voxel_list;
            voxblox::timing::Timer getbb("rrt/get_bb");
            voxblox::utils::getOrientedBoundingBox(*layer_, projection, bb_dim_x_, bb_dim_y_, bb_dim_z_, &block_voxel_list);
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
    float bb_dim_x_;
    float bb_dim_y_;
    float bb_dim_z_;

    // later std::vector<std::pair<float, float>> tire_pos_;
    double wheelbase_;
    double track_;
    float clearance_;
    float radius_;
    float car_angle_delta_;
    float ratio_trav_;
    bool relax_neighbor_search_;

    Eigen::Matrix<double, 3, 4> tire_offsets_;
    Eigen::Transform<float, 3, Eigen::Affine> bb_center_;

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
};
#endif  // VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_
