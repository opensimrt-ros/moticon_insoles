#!/usr/bin/env python3
# vim:fenc=utf-8

#
# @author      : frekle (frekle@bml01.mech.kth.se)
# @file        : publish_cop_tfs
# @created     : Thursday May 25, 2023 11:10:04 CEST
#
import rospy
import tf2_ros
from opensimrt_msgs.msg import Common
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from insoles_common import OpenSimTf

class CopTFPub:
    def __init__(self):
        self.l_frame = rospy.get_param("~left_cop_reference_frame", default="map")
        self.r_frame = rospy.get_param("~right_cop_reference_frame", default="map")
        self.foot_width = rospy.get_param("~foot_width", default=0.1)
        self.foot_length = rospy.get_param("~foot_length", default=0.27)
        self.copsub = ["",""]
        for i,side in enumerate(["left","right"]):
            #self.copsub[i] = rospy.Subscriber(side+'/cop', Common, callback=lambda x: self.callback_common(i, x), queue_size=1) ## here I probably want a geometry Point or a Point[]
            callback = lambda x, i=i: self.callback_common(i,x) ## i=i forces evaluation
            rospy.Subscriber(side+'/cop', Common, callback=callback, queue_size=1) ## here I probably want a geometry Point or a Point[]
        self.broadcaster = tf2_ros.TransformBroadcaster()
    def callback_common(self, side, common_msg):
        #print(side)
        msg_cop = common_msg.data
        #msg_cop = (0,0) ## needs to be obtained from common msg
        T = TransformStamped()
        H = Header()
        time_stamp = rospy.Time.now()

        H.stamp = time_stamp
        T.header.stamp = time_stamp
        if side: 
            H.frame_id = "right"
            x_axis_direction = 1
            T.child_frame_id = "right"
            T.header.frame_id = self.r_frame
        else:
            H.frame_id = "left"
            x_axis_direction = -1
            T.child_frame_id = "left"
            T.header.frame_id = self.l_frame
        
        T.transform.translation.x = self.foot_width/2*(msg_cop[1])*x_axis_direction ### need to check these because I am rotating them with the static transform afterwards...
        T.transform.translation.y = self.foot_length*(msg_cop[0] + 0.5) 
        T.transform.translation.z = 0
        T.transform.rotation = OpenSimTf.rotation
        #T.transform.rotation.x = 0
        #T.transform.rotation.y = 0.707
        #T.transform.rotation.z = 0.707
        #T.transform.rotation.w = 0
        self.broadcaster.sendTransform(T)
        

if __name__ == '__main__':
    try:
        rospy.init_node("publish_cops_from_tfs")
        cop_tf_pubs = CopTFPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



