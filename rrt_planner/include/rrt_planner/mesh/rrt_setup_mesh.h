//
// Created by jonasgerstner on 24.02.20.
//

#ifndef RRT_PLANNER_VOXBLOX_RRT_SETUP_H
#define RRT_PLANNER_VOXBLOX_RRT_SETUP_H
#include "rrt_planner/setup.h"
#include "ompl_stateval_mesh.h"

#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/geometry/mesh_conversion.h>

class MeshRrtSetup : public RrtSetup{
public:
    explicit MeshRrtSetup(float turning_radius) : RrtSetup(turning_radius){
        env_ = std::make_shared<Model>();
    }

    void setEnvironmentMesh(const pcl_msgs::PolygonMeshPtr &input_mesh){
        ROS_INFO("Received environment mesh.");
        Timer setup_fcl("fcl/setup");
        auto *mesh = new pcl::PolygonMesh;
        pcl::PolygonMeshConstPtr meshPtr(mesh);
        pcl_conversions::toPCL(*input_mesh, *mesh);

        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_pcl(new pcl::PointCloud<pcl::PointXYZ> );
        pcl::fromPCLPointCloud2(meshPtr->cloud, *vertices_pcl);
        vertices_.reserve(vertices_pcl->size());
        for (auto p : *vertices_pcl)
            vertices_.emplace_back(p.getVector3fMap());

        triangles_.reserve(meshPtr->polygons.size());
        for (pcl::Vertices poly : meshPtr->polygons)
            triangles_.emplace_back(fcl::Triangle(poly.vertices[0], poly.vertices[1], poly.vertices[2]));

        env_->beginModel(triangles_.size(), vertices_.size());
        env_->addSubModel(vertices_, triangles_);
        env_->endModel();
        setup_fcl.Stop();
        ROS_INFO("Finished building environment collision object.");
    }

    int getNumTrianglesEnv() const {return env_->num_tris;}

    void setFCLCollisionChecking(float current_height) {
        std::shared_ptr<FCLValidityChecker> validity_checker(
                new FCLValidityChecker(getSpaceInformation(), env_, cloud_, current_height, boundingbox_, carconfig_, search_radius_));

        setStateValidityChecker(ompl::base::StateValidityCheckerPtr(validity_checker));
//        si_->setMotionValidator(
//                ompl::base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::TsdfVoxel>(
//                        getSpaceInformation(), validity_checker)));
    }

protected:
    std::vector<fcl::Vector3f> vertices_;
    std::vector<fcl::Triangle> triangles_;

    std::shared_ptr<Model> env_;
};
#endif //RRT_PLANNER_VOXBLOX_RRT_SETUP_H
