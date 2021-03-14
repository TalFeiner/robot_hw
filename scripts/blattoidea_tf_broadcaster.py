#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


global seq, br_tf
seq = 0


def odom_cb(odom):
    global seq, br_tf
    tf_odom_msg = TransformStamped()
    tf_odom_msg.header.stamp = rospy.Time.now()
    tf_odom_msg.header.seq = seq
    tf_odom_msg.header.frame_id = "odom"
    tf_odom_msg.child_frame_id = "base_footprint"
    tf_odom_msg.transform.translation.x = odom.pose.pose.position.x
    tf_odom_msg.transform.translation.y = odom.pose.pose.position.y
    tf_odom_msg.transform.translation.z = 0.0
    tf_odom_msg.transform.rotation = odom.pose.pose.orientation

    try:
        br_tf.sendTransform(tf_odom_msg)
        seq += 1
    except tf2_ros.buffer_interface.TypeException as e:
        rospy.logerr(e)
        pass


rospy.init_node("blattoidea_tf_node")
br_tf = tf2_ros.TransformBroadcaster()
rospy.Subscriber("/odom", Odometry, odom_cb)
rospy.spin()
