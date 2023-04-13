#!/usr/bin/env python3
import os
from pathlib import Path

import socket
import moticon_insoles
import rospy
import tf2_ros
from std_msgs.msg import Header, Float32
from opensimrt_msgs.msg import Common
from opensimrt_msgs.srv import SetFileNameSrv, SetFileNameSrvResponse
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped, TransformStamped
from colorama import Fore
from sensor_msgs.msg import Imu
from math import pi
from copy import deepcopy
import numpy as np

GRAVITY = 9.80665

import time
import csv

from std_srvs.srv import Empty, EmptyResponse

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

def extract_insole_data(msg_insole):
    """Extract only the pressure and acc info from the large streaming data"""
    
    saving_data = [msg_insole.data_message.time, msg_insole.data_message.side,\
                   *msg_insole.data_message.pressure,\
                   *np.around(msg_insole.data_message.acceleration, decimals=3),\
                   *np.around(msg_insole.data_message.angular, decimals=3),\
                   msg_insole.data_message.total_force,\
                   *np.around(msg_insole.data_message.cop, decimals=5)]
        
    return saving_data
    

def insole_data_save(file_name, data):
    """save the insole data into a text file"""
    
    # create path if not exist
    directory = os.path.dirname(file_name)
    Path(directory).mkdir(parents=True, exist_ok=True)
    
    insole_file = open(file_name, 'w')  # open file for writing
    
    # save data into text file
    # write header
    header_str = ['Frame', 'side', 'P1', 'P2', 'P3',\
              'P4', 'P5', 'P6', 'P7',\
              'P8', 'P9', 'P10', 'P11',\
              'P12', 'P13', 'P14', 'P15',\
              'P16', 'acc1', 'acc2', 'acc3', 'ang1', 'ang2', 'ang3',\
              'totalForce', 'cop1', 'cop2']
           
    try:
        for header_name in header_str:  # write header
            insole_file.write(header_name)
            insole_file.write(' ')
        insole_file.write('\n')
        
        c = len(data[0])  # get col number
        for row in data:  # write data
            for col in range(0, c):
                try:
                    insole_file.write(str(row[col]))
                    insole_file.write(' ')
                except:
                    pass
            insole_file.write('\n')
             
    finally:
        insole_file.close()


class InsoleSrv:
    def __init__(self):
        self.recording = False
        self.savedict_list = []
        self.file_name = "/tmp/insole.txt"

    def turn_on_recording(self, req):
        rospy.loginfo("Started recording")
        self.recording = True
        return EmptyResponse()

    def turn_off_recording(self, req):
        rospy.loginfo("Stopped recording")
        self.recording = False
        return EmptyResponse()

    def save(self, req):
        insole_data_save(self.file_name, self.savedict_list)
        rospy.loginfo("Recorded data saved in %s"%self.file_name)
        return EmptyResponse()

    def clear(self, req):
        self.savedict_list = []
        rospy.loginfo("Recorded data cache cleared.")
        return EmptyResponse()

    def setfilename(self, req):
        rospy.loginfo("Using directory for savedata: %s"%req.path)
        rospy.loginfo("Using filename for savedata: %s"%req.name)
        self.file_name = req.path + "/" + req.name + "_insole.txt"
        return SetFileNameSrvResponse()

    def run_server(self, server_name = "", port = 9999):
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
        
        execution_timer = rospy.Publisher("/insoles", Float32, queue_size=1)
        
        insole_rate = rospy.get_param("~insole_rate")
        node_freq = 2*insole_rate
        rospy.loginfo(f"Insole rate read freq set to {insole_rate}")
        rospy.loginfo(f"Node freq will be set to {node_freq}")
        rate = rospy.Rate(node_freq) ## if we read at 100 hertz from 2 sensors this should be enough?
        initialized = False
        initial_time = None
        last_time = [None,None] ## left and right have different counters!
        thistime = [None,None]
     

        #recording service
        s = rospy.Service('~record', Empty, self.turn_on_recording)
        s1 = rospy.Service('~stop', Empty, self.turn_off_recording)
        s2 = rospy.Service('~save', Empty, self.save)
        s3 = rospy.Service('~setfilename', SetFileNameSrv, self.setfilename)
        s4 = rospy.Service('~clear', Empty, self.clear)
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

                    msg.ParseFromString(msg_buf) # TODO: not sure this erases everything that was already inside moticon_insoles.proto_s.MoticonMessage() need to measure speed gains to see if this makes sense

                    #create dict we want to save later:
                    if self.recording:
                        #rospy.loginfo("REC\r")
                        try:
                            self.savedict_list.append(extract_insole_data(msg))

                        except Exception as exc:
                            rospy.logerr("could not create savedict. data for this frame will not be saved.%s"%exc)



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
                        #rospy.logwarn("no time in message!")
                        pass
                    else:
                        thistime[side] = msg.data_message.time
                        if thistime[side] and last_time[side]:
                            timediff = thistime[side] - last_time[side]   
                            rospy.logdebug("time counter %d, time difference %d"%(thistime[side], timediff))
                            tmsg =  Float32(timediff)
                            ips.time[side].publish(tmsg)
                        last_time = deepcopy(thistime)

                    if not msg.data_message.total_force:
                        pass
                        #rospy.logwarn("no total_force. not publishing force or wrench topics")
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
                        pass
                        #rospy.logwarn_once("no cop. not publishing ...")
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
                        pass
                        #rospy.logwarn_once("no pressure data. not publishing")
                    else:
                        pmsg = Common(h, msg.data_message.pressure)
                        ips.pressure[side].publish(pmsg)

                    if not msg.data_message.angular or not msg.data_message.acceleration:
                        pass
                        #rospy.logwarn_once("no angular or acceleration data. cannot publishing imu_msg")
                    else:
                        imsg = convert_to_imu(h, msg.data_message.angular, msg.data_message.acceleration)
                        ips.imu[side].publish(imsg)

                    rate.sleep()
                    toc = time.perf_counter()
                    execution_timer.publish((toc-tic)*1000)
                    #rospy.loginfo(f"time it took to run over loop once {(toc - tic)*1000:0.4f} ms")
                raise rospy.ROSInterruptException("This is fine. It's the way to close this otherwise it will run forever.")
            finally:
                connection.close()

#moticon_insoles.run_server()
insrv = InsoleSrv()
insrv.run_server()

