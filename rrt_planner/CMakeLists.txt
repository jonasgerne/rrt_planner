cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
find_package(FCL 0.6.1 REQUIRED)
find_package(ccd REQUIRED)
find_package(PCL REQUIRED geometry)
#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_common
        src/rrt_planner.cpp
        src/ompl_rrt.cpp)

target_link_libraries(${PROJECT_NAME}_common ${OMPL_LIBRARIES})

add_library(${PROJECT_NAME}_voxel
        src/voxblox/ompl_rrt_voxblox.cpp
        src/voxblox/rrt_planner_voxblox.cpp
)

target_link_libraries(${PROJECT_NAME}_voxel ${PROJECT_NAME}_common ${OMPL_LIBRARIES})

add_library(${PROJECT_NAME}_mesh
        src/mesh/ompl_rrt_mesh.cpp
        src/mesh/rrt_planner_mesh.cpp
        src/timing.cc
        )

target_link_libraries(${PROJECT_NAME}_mesh
        ${PROJECT_NAME}_common
        ${OMPL_LIBRARIES}
        ${FCL_LIBRARIES}
        ${CCD_LIBRARIES}
        ${PCL_LIBRARIES})
############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_voxel_node
        src/voxblox/rrt_planner_node_voxel.cpp
)
target_link_libraries(${PROJECT_NAME}_voxel_node
        ${PROJECT_NAME}_voxel)

cs_add_executable(${PROJECT_NAME}_mesh_node
        src/mesh/rrt_planner_node_mesh.cpp
        )

target_link_libraries(${PROJECT_NAME}_mesh_node
        ${PROJECT_NAME}_mesh)

##########
# EXPORT #
##########
cs_install()
cs_export()
