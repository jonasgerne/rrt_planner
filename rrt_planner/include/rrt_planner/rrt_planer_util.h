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
};

struct CarConfig{
    float wheelbase;
    float track;
    float clearance;
};
#endif //RRT_PLANNER_RRT_PLANER_UTIL_H
