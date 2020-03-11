#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

if __name__ == '__main__':
    rospy.init_node("vis_box")

    marker_pub = rospy.Publisher('bb_viz', Marker, queue_size=1)
    pose_pub = rospy.Publisher('bb_pose', PoseStamped, queue_size=1)
    bounding = [5.0, 2.2, 1.4]

    input = """0.00985353   -0.569643   -0.821833    -198.186
 0.00490808   -0.821835    0.569704      210.28
  -0.999939 -0.00964722 -0.00530212    0.985281
4.58323e-41           0           0           0"""
    mat = np.asarray(input.split()).reshape(4, 4)
    mat = mat.astype(float)
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.ns = "bb"
    msg.type = Marker.CUBE
    msg.action = Marker.ADD
    msg.pose.position.x = mat[0, 3]
    msg.pose.position.y = mat[1, 3]
    msg.pose.position.z = mat[2, 3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    msg.scale.x = bounding[0]
    msg.scale.y = bounding[1]
    msg.scale.z = bounding[2]
    msg.color.a = 0.5
    msg.color.r = 1.0
    msg.color.g = 0.0
    msg.color.b = 0.0

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = mat[0, 3]
    pose.pose.position.y = mat[1, 3]
    pose.pose.position.z = mat[2, 3]

    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    


    while not rospy.is_shutdown():
        marker_pub.publish(msg)
        pose_pub.publish(pose)
        rospy.sleep(1)

