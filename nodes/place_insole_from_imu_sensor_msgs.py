#!/usr/bin/env python3
# vim:fenc=utf-8

#
# @author      : frekle (frekle@bml01.mech.kth.se)
# @file        : place_insole_from_imu_sensor_msgs
# @created     : Thursday May 10, 2024 18:30:53 CEST
#


import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

class ImuTFPub:
    def __init__(self):
        self.insole_parent_frame = rospy.get_param("~foot_center_name")
        self.side_of_insole = rospy.get_param("~side_of_insole")
        rospy.Subscriber("imu", Imu, callback=self.callback, queue_size=1)
        self.broadcaster = tf2_ros.TransformBroadcaster()
    def callback(self, imu_msg):
        #print(side)
        T = TransformStamped()
        T.header = imu_msg.header

        T.header.frame_id = self.insole_parent_frame
        T.child_frame_id = self.side_of_insole

        T.transform.rotation = imu_msg.orientation
        self.broadcaster.sendTransform(T)


if __name__ == '__main__':
    try:
        rospy.init_node("publish_tfs_from_imu_from_insole")
        cop_tf_pubs = ImuTFPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




