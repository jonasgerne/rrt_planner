//
// Created by jonasgerstner on 24.02.20.
//
#include <ros/ros.h>
#include "rrt_planner/voxblox/rrt_planner_voxblox.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_planner");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    VoxbloxRrtPlanner p(nh, nh_private);
    ROS_INFO("Initialized RRT Planner Voxblox node.");

    ros::spin();
    return 0;
}
