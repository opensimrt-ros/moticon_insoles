#!/usr/bin/env python3
import socket
import moticon_insoles
import rospy
import tf2_ros
from std_msgs.msg import Header, Float32
from opensimrt_msgs.msg import Common 
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped, TransformStamped
from colorama import Fore
from sensor_msgs.msg import Imu
from math import pi
from copy import deepcopy

GRAVITY = 9.80665

import time


class InsolePublishers():
    def __init__(self):
        self.pressure = ["",""]
        self.force = ["",""]
        self.time = ["",""]
        self.cop = ["",""]
        self.wrench = ["",""]
        self.imu = ["",""]

def convert_to_imu(h, angular_velocity,linear_acceleration):
    imu_msg = Imu()   
    imu_msg.header = h
    #the sensor for this insole is the LSM6DSL so in g and degrees/second
    if len(angular_velocity) >=2:
        imu_msg.angular_velocity.x = angular_velocity[0]/180.0*pi
        imu_msg.angular_velocity.y = angular_velocity[1]/180.0*pi
        imu_msg.angular_velocity.z = angular_velocity[2]/180.0*pi
    if len(linear_acceleration) >=2:
        imu_msg.linear_acceleration.x = linear_acceleration[0]/GRAVITY
        imu_msg.linear_acceleration.y = linear_acceleration[1]/GRAVITY
        imu_msg.linear_acceleration.z = linear_acceleration[2]/GRAVITY

    return imu_msg

def run_server(server_name = "", port = 9999):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (server_name, port)
    rospy.loginfo('Starting Moticon Insole server up on {} port {}'.format(server_name, port))
    sock.bind(server_address)
    sock.listen(1)
    
    # Create ros stuff
    rospy.init_node("moticon_insoles", anonymous=True)
    ips = InsolePublishers()
    
    l_frame = rospy.get_param("~left_cop_reference_frame")
    r_frame = rospy.get_param("~right_cop_reference_frame")
    for i,side in enumerate(["left","right"]):
        ips.pressure[i] = rospy.Publisher(side+'/pressure', Common, queue_size=1)
        ips.force[i] = rospy.Publisher(side+'/force', Float32, queue_size=1)
        ips.time[i] = rospy.Publisher(side+'/time_diff', Float32, queue_size=1)
        ips.cop[i] = rospy.Publisher(side+'/cop', Common, queue_size=1) ## here I probably want a geometry Point or a Point[]
        ips.wrench[i] = rospy.Publisher(side+'/wrench', WrenchStamped, queue_size=1)
        ips.imu[i] = rospy.Publisher(side+'/imu_raw', Imu, queue_size=1)
    broadcaster = tf2_ros.TransformBroadcaster()

    
    rate = rospy.Rate(1000) ## if we read at 100 hertz from 2 sensors this should be enough?
    initialized = False
    initial_time = None
    last_time = [None,None] ## left and right have different counters!
    thistime = [None,None]
    msg = moticon_insoles.proto_s.MoticonMessage() ## maybe this can be outside the loop
    while not rospy.is_shutdown(): ## maybe while ros ok
        rospy.loginfo('Waiting for a connection...')
        connection, client_address = sock.accept()
        rospy.loginfo('Connection created.')
        rospy.logdebug('client connected: {}'.format(client_address))
        try:
            while not rospy.is_shutdown(): ## we maybe want to rate limit this.
                tic = time.perf_counter()
                try:
                    msg_buf = moticon_insoles.get_message(connection)
                except moticon_insoles.ConnectionClosed as e:
                    rospy.logerr(e)
                    break

                msg.ParseFromString(msg_buf)
                if not initialized:
                    initial_time = msg.data_message.time
                    initialized = True

                # Now handle the message
                color = ""
                if msg.data_message.side:
                    color =Fore.CYAN # RIGHT SIDE ## consider adding for debug
                else:
                    color =Fore.LIGHTCYAN_EX
                rospy.logdebug(color+str(msg)) ## consider adding for debug

                rospy.logdebug(color+str(msg.data_message))
                rospy.logdebug(color+str(msg.data_message.pressure))
                rospy.logdebug(color+str(msg.data_message.total_force)) ## to display in Rviz we need to use a WrenchStamped
                rospy.logdebug(color+str(msg.data_message.cop)) ## maybe this is a geometry/Point

                ### this will be relevant for the sensors/imu raw publisher. we also need to make sure it is using SI and they are the same as ROS's definitions
                ## I will also need this if we have an inclined plane!!!
                rospy.logdebug(color+str(msg.data_message.angular))
                rospy.logdebug(color+str(msg.data_message.acceleration))

                rospy.logdebug(color+str(msg.data_message.time)) ## not sure if I need this guy
                #rospy.logdebug(color+str(msg.data_message.temperature) ## in our device this is always zero
                rospy.logdebug(color+str(msg.data_message.service_id)) ## not sure what this is either
                rospy.logdebug(color+str(msg.data_message.side))


                
                # Publish these guys
                h = Header()
                time_stamp = rospy.Time.now()
                h.stamp = time_stamp
                
                ## now I need to publish it to the right side. 
                ## maybe this is wrong and I need to publish them both at the same time, but since I receive a message which is from either one side or the other, than the other side's info would be zero, so I didn't solve anything by doing this, I just pushed the problem further. At some point I need to remember which side is doing what. I can't rely on tf for this, so maybe I need to remember the latest values, update them here and publish both at the same time?

                side = msg.data_message.side
                
                t = TransformStamped()
                if msg.data_message.side: ## or the other way around, needs checking
                    h.frame_id = "right"
                    t.child_frame_id = "right"
                    offset = -0.3
                    x_axis_direction = 1
                    t.header.frame_id = r_frame

                else:
                    h.frame_id = "left"
                    t.child_frame_id = "left"
                    x_axis_direction = -1
                    offset = 0.3
                    t.header.frame_id = l_frame
                
                
                pressure = 0
                force = 0
                cop = (0,0)
                if not msg.data_message.time:
                    rospy.logwarn("no time in message!")
                else:
                    thistime[side] = msg.data_message.time
                    if thistime[side] and last_time[side]:
                        timediff = thistime[side] - last_time[side]   
                        rospy.logdebug("time counter %d, time difference %d"%(thistime[side], timediff))
                        tmsg =  Float32(timediff)
                        ips.time[side].publish(tmsg)
                    last_time = deepcopy(thistime)

                if not msg.data_message.total_force:
                    rospy.logwarn("no total_force. not publishing force or wrench topics")
                else:
                    fmsg = Float32(msg.data_message.total_force)
                    try:
                        ips.force[side].publish(fmsg)
                    except Exception as e:
                        rospy.logerr("total force looks like this: %s and I can't publish it!! %s"%(msg.data_message.total_force, e))
                    force = Vector3(y=msg.data_message.total_force)
                    wren = Wrench(force=force) #force, torque
                    wmsg = WrenchStamped(h,wren)
                    
                    ips.wrench[side].publish(wmsg)
                ## we also want to send a tf for the cop

                

                if not msg.data_message.cop:
                    rospy.logwarn_once("no cop. not publishing ...")
                else:
                    cmsg = Common(h, msg.data_message.cop)
                    ips.cop[side].publish(cmsg)
                    t.header.stamp = time_stamp
                    t.transform.translation.x = msg.data_message.cop [1]/5*x_axis_direction ### need to check these because I am rotating them with the static transform afterwards...
                    t.transform.translation.y = msg.data_message.cop [0]/5 + 0.1 
                    t.transform.translation.z = 0
                    t.transform.rotation.x = 0
                    t.transform.rotation.y = 0.707
                    t.transform.rotation.z = 0.707
                    t.transform.rotation.w = 0
                    broadcaster.sendTransform(t)

                if not msg.data_message.pressure:
                    rospy.logwarn_once("no pressure data. not publishing")
                else:
                    pmsg = Common(h, msg.data_message.pressure)
                    ips.pressure[side].publish(pmsg)

                if not msg.data_message.angular or not msg.data_message.acceleration:
                    rospy.logwarn_once("no angular or acceleration data. cannot publishing imu_msg")
                else:
                    imsg = convert_to_imu(h, msg.data_message.angular, msg.data_message.acceleration)
                    ips.imu[side].publish(imsg)

                #rate.sleep()
                toc = time.perf_counter()
                #rospy.loginfo(f"time it took to run over loop once {(toc - tic)*1000:0.4f} ms")
            raise rospy.ROSInterruptException()
        finally:
            connection.close()

#moticon_insoles.run_server()
run_server()

