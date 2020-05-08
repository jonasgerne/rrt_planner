//
// Created by jonasgerstner on 25.02.20.
//

#ifndef RRT_PLANNER_RRT_PLANER_UTIL_H
#define RRT_PLANNER_RRT_PLANER_UTIL_H
struct BBConfig{
    float theta;
    float dim_x;
    float dim_y;
    float dim_z;
    float offset_x;
};

struct CarConfig{
    float wheelbase;
    float track;
    float clearance;
};

struct ValConfig{
    float radius;
    float ratio_trav;
    float car_angle_delta;
    bool debug_fcl;
    bool relax_neighbor_search;
};
#endif //RRT_PLANNER_RRT_PLANER_UTIL_H
