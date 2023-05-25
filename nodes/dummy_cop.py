#!/usr/bin/env python3
# vim:fenc=utf-8

#
# @author      : frekle (frekle@bml01.mech.kth.se)
# @file        : dummy_cop
# @created     : Thursday May 25, 2023 11:52:53 CEST
#
import rospy
from opensimrt_msgs.msg import Common
from std_msgs.msg import Header
import random, math

limits = [-0.5, 0.5]

def get_rand_foot():
    x = random.uniform(limits[0], limits[1])
    y = random.uniform(limits[0], limits[1])
    return (x, y)

def rand_circle_motion():
    angle = 0.0
    print(angle)
    while True:
        angle += random.uniform(0.1, 0.5)  # Increase the angle by a random amount
        x = limits[1] * math.cos(angle)  # Calculate x-coordinate using cosine
        y = limits[1] * math.sin(angle)  # Calculate y-coordinate using sine
        yield (x, y)

def get_rand_circle_motion():
    return next(rand_circle_motion())

def main():
    rospy.init_node("dummy_cop")
    pub = rospy.Publisher("foot_cop", Common, queue_size=1)
    frame = rospy.get_param("~frame_id", "a_foot")
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
        #cop = get_rand_circle_motion()
        cop = getter_fun()
        #print(cop)
        cop_msg = Common(h,cop)
        
        pub.publish(cop_msg)
        rate.sleep()

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass



