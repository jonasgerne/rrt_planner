#!/usr/bin/env python
import rospy
from rrt_planner_msgs.srv import *
from geometry_msgs.msg import PoseStamped

import pickle


def create_poses():
    start = PoseStamped()
    goal = PoseStamped()
    start.header.frame_id = "world"
    start.header.stamp = rospy.Time.now()

    start.pose.position.x = -201.687850952
    start.pose.position.y = 225.336380005
    start.pose.position.z = 0.0

    start.pose.orientation.x = 0.0
    start.pose.orientation.y = 0.0
    start.pose.orientation.z = -0.705527220499
    start.pose.orientation.w = 0.70868282125

    # start.pose.position.x = -207.399
    # start.pose.position.y = 216.439
    # start.pose.position.z = 0.3
    #
    # start.pose.orientation.x = 0.0
    # start.pose.orientation.y = 0.0
    # start.pose.orientation.z = 0.916
    # start.pose.orientation.w = -0.401

    goal.header.frame_id = "world"
    goal.header.stamp = rospy.Time.now()

    # goal.pose.position.x = -201.668
    # goal.pose.position.y = 204.369
    # goal.pose.position.z = 0.0
    #
    # goal.pose.orientation.x = 0.0
    # goal.pose.orientation.y = 0.0
    # goal.pose.orientation.z = -0.704
    # goal.pose.orientation.w = 0.710
    goal.pose.position.x = -200.803
    goal.pose.position.y = 204.504
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = -0.709
    goal.pose.orientation.w = 0.706
    return start, goal


def call_rrt_planner(start_pose, goal_pose, max_time=None):
    rospy.wait_for_service("/rrt_planner/plan")
    try:
        rrt_service_proxy = rospy.ServiceProxy('/rrt_planner/plan', PlannerService)
        res = rrt_service_proxy(start_pose, goal_pose, None, max_time)
        return res.success, res.path, res.path_length
    except rospy.ServiceException, e:
        print("Service call failed")
        return False, None, 0.0


if __name__ == '__main__':
    rospy.init_node("rrt_planner_caller")

    start_pub = rospy.Publisher('rrt_start', PoseStamped, latch=True, queue_size=1)
    goal_pub = rospy.Publisher('rrt_goal', PoseStamped, latch=True, queue_size=1)

    start, goal = create_poses()

    start_pub.publish(start)
    goal_pub.publish(goal)

    success, path, length = call_rrt_planner(start, goal, 10.0)

    # results = []
    # for t in [0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0]:
    #     for i in range(20):
    #         print("Call {} for max_time {}".format(i, t))
    #         success, path, length = call_rrt_planner(start, goal, t)
    #         results.append({'max_time': t, 'trial': i, 'success': success, 'length': length})
    #
    # filename = "test_{}.pickle".format(rospy.Time.now())
    # with open(filename, 'w') as f:
    #     pickle.dump(results, f)
    #rospy.spin()

