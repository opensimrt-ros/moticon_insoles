#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
trigger2 = rospy.ServiceProxy("/inverse_kinematics_from_file/start", Empty)
trigger1 = rospy.ServiceProxy("/moticon_insoles/start_playback", Empty)

def start_multi(req):
    rospy.loginfo("calling trigger1")
    trigger1()
    rospy.loginfo("calling trigger2")
    trigger2()
    return EmptyResponse()

rospy.init_node("start_multi");
rospy.Service("~trigger", Empty, start_multi)
rospy.spin()

