#!/usr/bin/env python3
# vim:fenc=utf-8

#
# @author      : frekle (frekle@bml01.mech.kth.se)
# @file        : dummy_wrench
# @created     : Thursday May 25, 2023 17:46:42 CEST
#
import rospy
from opensimrt_msgs.msg import Common
from geometry_msgs.msg import WrenchStamped, Vector3
from std_msgs.msg import Header
import random, math

limits = [-5, 500]

def get_rand_foot():
    x = random.uniform(limits[0], limits[1])
    y = random.uniform(limits[0], limits[1])
    z = random.uniform(limits[0], limits[1])
    return (x, y, z)

def rand_circle_motion():
    angle = 0.0
    print(angle)
    while True:
        angle += random.uniform(0.1, 0.5)  # Increase the angle by a random amount
        x = limits[1] * math.cos(angle)  # Calculate x-coordinate using cosine
        y = limits[1] * math.sin(angle)  # Calculate y-coordinate using sine
        z = limits[1]   # 
        yield (x, y, z)

def get_rand_circle_motion():
    return next(rand_circle_motion())

def main():
    rospy.init_node("dummy_wrench")
    pub = rospy.Publisher("foot_wrench", WrenchStamped, queue_size=1)
    frame = rospy.get_param("~frame_id", "map")
    use_circle_motion = rospy.get_param("~circle_motion", True)
    if use_circle_motion:
        getter_fun = rand_circle_motion().__next__
    else:
        getter_fun = get_rand_foot
    rate = rospy.Rate(rospy.get_param("rate", 10))
    while not rospy.is_shutdown():
        h = Header()
        h.frame_id = frame
        h.stamp = rospy.Time.now()
        #wr = get_rand_circle_motion()
        wr = getter_fun()
        v = Vector3()
        v.x = 0 #wr[0]
        v.y = wr[1] + limits[1]
        v.z = 0 #wr[2]
        #print(wr)
        wr_msg = WrenchStamped()
        wr_msg.header = h
        wr_msg.wrench.force = v
        tt = Vector3()
        tt.x = wr[1]/10000
        tt.y = wr[0]/10000
        tt.z = wr[2]/10000
        
        wr_msg.wrench.torque = tt
        
        pub.publish(wr_msg)
        rate.sleep()

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass



