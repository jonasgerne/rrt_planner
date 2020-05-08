#!/usr/bin/env python
import rospy
from rrt_planner_msgs.srv import *
from geometry_msgs.msg import PoseStamped
import re
import pickle
import sys
import numpy as np
from tf.transformations import quaternion_matrix, euler_from_matrix, quaternion_from_matrix
from datetime import datetime
import math

from visualization_msgs.msg import Marker

sys.path.append("/home/jonasgerstner/Documents/CARLA/PythonAPI/carla/dist")
import carla

rear_ax_to_center = np.array([[1.0, 0.0, 0.0, 1.41], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.2], [0.0, 0.0, 0.0, 1.0]])

deg = lambda x: (x / np.pi) * 180.0

pose_string_start = """Position(-93.962, -121.545, 0.000), Orientation(0.000, 0.000, -0.048, 0.999)"""
pose_string_goal = """Position(-66.079, -123.021, 0.000), Orientation(0.000, 0.000, -0.034, 0.999)"""

pose_string_cone_start = """Position(-66.648, 94.950, 0.000), Orientation(0.000, 0.000, -0.367, 0.930)"""
pose_string_cone_goal = """Position(-24.774, 97.492, 0.000), Orientation(0.000, 0.000, 0.483, 0.876)"""

fuel_start = """Position(-44.699, -175.009, 0.000), Orientation(0.000, 0.000, 0.528, 0.849)"""
fuel_stop = """Position(-14.502, -164.697, 0.000), Orientation(0.000, 0.000, -0.031, 1.000)"""


class PlanService(object):
    def __init__(self):
        client = carla.Client("localhost", 2000)
        self.world = client.get_world()
        self.car_bp = self.world.get_blueprint_library().find("vehicle.tesla.model3")

        self.rrt_service_proxy = rospy.ServiceProxy('/rrt_planner/plan', PlannerService)

        self.marker_pub = rospy.Publisher('bb_viz', Marker, queue_size=1, latch=True)

    def parse_pose_string(self, pose):
        """
        helper function to convert "2D nav goals" from rviz to a ROS pose
        :param pose: str A string containing position and orientation of a pose as quaternion (should contain 7 floating
        point numbers
        :return: geometry_msgs.msg.PoseStamped
        """
        pose_str = re.findall(r"[-+]?\d*\.\d+|\d+", pose)
        pose = map(float, pose_str)
        ros_pose = PoseStamped()

        ros_pose.header.frame_id = "world"
        ros_pose.header.stamp = rospy.Time.now()

        ros_pose.pose.position.x = pose[0]
        ros_pose.pose.position.y = pose[1]
        ros_pose.pose.position.z = pose[2] + 0.2

        ros_pose.pose.orientation.x = pose[3]
        ros_pose.pose.orientation.y = pose[4]
        ros_pose.pose.orientation.z = pose[5]
        ros_pose.pose.orientation.w = pose[6]
        return ros_pose

    def create_marker_msg(self, pose, success):
        msg = Marker()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.ns = "bb"
        msg.type = Marker.CUBE
        msg.action = Marker.ADD

        se3_pose = self.pose_to_se3(pose).dot(rear_ax_to_center)

        msg.pose.position.x = se3_pose[0, 3]
        msg.pose.position.y = se3_pose[1, 3]
        msg.pose.position.z = se3_pose[2, 3] + 0.9

        q = quaternion_from_matrix(se3_pose)

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        msg.scale.x = 5.218
        msg.scale.y = 2.689
        msg.scale.z = 1.0
        msg.color.a = 0.5
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        if success:
            msg.color.g = 1.0
        else:
            msg.color.r = 1.0
        return msg

    @staticmethod
    def pose_to_se3(pose):
        quat = np.array(
            [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        tf_mat = quaternion_matrix(quat)
        tf_mat[0, 3] = pose.pose.position.x
        tf_mat[1, 3] = pose.pose.position.y
        tf_mat[2, 3] = pose.pose.position.z

        return tf_mat

    def convert_pose_to_carla(self, pose):
        se3_pose = self.pose_to_se3(pose)

        to_center = se3_pose.dot(rear_ax_to_center)

        location = carla.Location(x=to_center[0, 3], y=-to_center[1, 3], z=to_center[2, 3])

        r, p, y = euler_from_matrix(to_center)

        rotation = carla.Rotation(roll=math.degrees(r), pitch=math.degrees(-p), yaw=math.degrees(-y))

        return carla.Transform(location, rotation)

    def check_poses_in_carla(self, path):
        for p in path.poses:
            #rospy.sleep(1.0)
            transform = self.convert_pose_to_carla(p)
            res = self.world.try_spawn_actor(self.car_bp, transform)
            try:
                self.marker_pub.publish(self.create_marker_msg(p, bool(res)))
            except rospy.ROSException as e:
                res.destroy()
            if not res:
                print("collision at ")
                print(transform)
                return False
            else:
                res.destroy()
        return True

    def call_rrt_planner(self, start_pose, goal_pose, max_time=None):
        rospy.wait_for_service("/rrt_planner/plan")
        try:
            res = self.rrt_service_proxy(start_pose, goal_pose, None, max_time)
            return res.success, res.path, res.path_length
        except rospy.ServiceException, e:
            print("Service call failed")
            return False, None, 0.0


if __name__ == '__main__':
    rospy.init_node("rrt_planner_caller")

    test = sys.argv[1]
    if test:
        timestamp = datetime.now()
        print(timestamp.strftime("%d_%m_%Y_%H_%M_%S"))
        start_pub = rospy.Publisher('rrt_start', PoseStamped, latch=True, queue_size=1)
        goal_pub = rospy.Publisher('rrt_goal', PoseStamped, latch=True, queue_size=1)

        plan_service = PlanService()
        start = plan_service.parse_pose_string(pose_string_start)
        goal = plan_service.parse_pose_string(pose_string_goal)

        start_pub.publish(start)
        goal_pub.publish(goal)

        results = []
        t = 20.0
        for r in [10]:  # [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]:
            for i in range(r):
                print("Call {} for max_time {} samples: {}".format(i+1, t, r))
                success, path, length = plan_service.call_rrt_planner(start, goal, t)
                coll_free = False
                if success:
                    print("Length: {}".format(length))
                    coll_free = plan_service.check_poses_in_carla(path)

                if not coll_free:
                    with open("paths/path_{}_{}.pickle".format(test, i), 'w') as p:
                        pickle.dump(path, p)
                results.append({'test': test, 'max_time': t, 'trial': i, 'plan_success': success, 'samples': r, 'carla_success': coll_free, 'length': length})

        filename = "{}.pickle".format(test)
        with open(filename, 'w') as f:
            pickle.dump(results, f)
    # rospy.spin()
