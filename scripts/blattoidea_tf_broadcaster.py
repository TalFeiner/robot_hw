#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


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
        seq += seq
        if seq < 10:
            map_flag = Bool()
            map_flag.data = True
            map_tf_frame_exist.publish(map_flag)
    except tf2_ros.buffer_interface.TypeException as e:
        tf_odom_msg.header.frame_id = "world"
        br_tf.sendTransform(tf_odom_msg)
        seq += 1
        rospy.logerr_once(e)
        if seq < 10:
            map_flag = Bool()
            map_flag.data = False
            map_tf_frame_exist.publish(map_flag)
        pass


rospy.init_node("blattoidea_tf_node")
br_tf = tf2_ros.TransformBroadcaster()
map_tf_frame_exist = rospy.Publisher("/map_tf_frame_exist_flag", Bool, queue_size=1)
rospy.Subscriber("/odom", Odometry, odom_cb)
rospy.spin()
