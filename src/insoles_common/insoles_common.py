#!/usr/bin/env python3
#import moticon_insoles

print("Loaded insoles_common.py")

import os
import insoles_common
from pathlib import Path

import socket
import moticon_insoles
import rospy
import tf2_ros
from std_msgs.msg import Header, Float32
from opensimrt_msgs.msg import Common
from opensimrt_msgs.srv import SetFileNameSrv, SetFileNameSrvResponse
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped, TransformStamped, Transform
from insole_msgs.msg import InsoleSensorStamped
from colorama import Fore
from sensor_msgs.msg import Imu
from math import pi
from copy import deepcopy
import numpy as np
import abc
from abc import ABC, abstractmethod

GRAVITY = 9.80665

import time
import csv

from std_srvs.srv import Empty, EmptyResponse

import buffer

OpenSimTf = Transform()
OpenSimTf.rotation.x = 0
OpenSimTf.rotation.y = 0.707
OpenSimTf.rotation.z = 0.707
OpenSimTf.rotation.w = 0

class InsolePublishers():
    def __init__(self):
        self.pressure = ["",""]
        self.force = ["",""]
        self.time = ["",""]
        self.cop = ["",""]
        self.wrench = ["",""]
        self.imu = ["",""]
        self.insole = ["",""]

def convert_to_imu(h, angular_velocity,linear_acceleration):
    imu_msg = Imu()   
    imu_msg.header = h
    #the sensor for this insole is the LSM6DSL so in g and degrees/second
    if len(angular_velocity) == 3:
        imu_msg.angular_velocity.x = angular_velocity[0]/180.0*pi
        imu_msg.angular_velocity.y = angular_velocity[1]/180.0*pi
        imu_msg.angular_velocity.z = angular_velocity[2]/180.0*pi
    if len(linear_acceleration) == 3:
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

class InsoleDataGetter(ABC):
    """Abstract class to interface between reading from file or sensor"""

    @abc.abstractproperty
    def ok():
        pass
    @abstractmethod
    def start_listening(self):
        pass
    @abstractmethod
    def get_data(self):
        pass
    @abstractmethod
    def __del__(self):
        pass

class InsoleDataFromSocket(InsoleDataGetter):
    """Sensor reader class"""
    def __init__(self, server_name = "", port = 9999):
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = (server_name, port)
        rospy.loginfo('Starting Moticon Insole server up on {} port {}'.format(server_name, port))
        self.sock.bind(server_address)
        self.sock.listen(1)
        self.connection = None
        self.connection_ok = True

    @property
    def ok(self):
        #print("called ok")
        return self.connection_ok

    def start_listening(self):
        rospy.loginfo('Waiting for a connection...')
        self.connection, client_address = self.sock.accept()
        rospy.loginfo('Connection created.')
        rospy.logdebug('client connected: {}'.format(client_address))

    def get_data(self):
        msg = moticon_insoles.proto_s.MoticonMessage() ## maybe this can be outside the loop
        try:
            msg_buf = moticon_insoles.get_message(self.connection)
        except moticon_insoles.ConnectionClosed as e:
            rospy.logerr(e)
            self.connection_ok = False
            print("Connection Closed. ")
            return 

        msg.ParseFromString(msg_buf) # TODO: not sure this erases everything that was already inside moticon_insoles.proto_s.MoticonMessage() need to measure speed gains to see if this makes sense
        side = msg.data_message.side
        msg_time = msg.data_message.time
        if msg_time == 0: ## we want to catch that weird message with Frame = 0
            rospy.logwarn("Got this message with a strange time_stamp:\n%s"%msg)
        msg_total_force = msg.data_message.total_force
        msg_cop = msg.data_message.cop
        msg_ang = msg.data_message.angular
        msg_acc = msg.data_message.acceleration
        msg_pres = msg.data_message.pressure
        return msg_time, side, msg_pres, msg_acc, msg_ang, msg_total_force, msg_cop 

    def __del__(self):
        self.connection.close()

class InsoleDataFromFile(InsoleDataGetter):
    """Data from file class"""
    def __init__(self, filename = ""):
        self.filename = filename
        self.file = None
        self.reader = None

    def start_listening(self):
        #maybe opens file and we have a getline thing going
        print(self.filename)
        self.file = open(self.filename, "r")
        self.reader = csv.DictReader(self.file, delimiter=" ")
        #print(next(self.reader))

    @property
    def ok(self):
        #maybe checks if there is still things to be read.
        if self.file:
            return True
        else:
            return False

    def get_data(self):
        frame_msg = next(self.reader)
        #print(frame_msg)
        def get_prop(props): ## If I try to access a property that was not saved I get a key error. Since you can disable pressure sensors
            ##and accelerometers and still have useful insole data, we better check if that was saved or not. 
            ##the rest of the function is getting either tuples for IMU or pressure data and making sure they are numbers (csv reads them as strings by default)
            for prop in props:
                if not prop in frame_msg.keys():
                    return None
            if len(props) == 1:
                return float(frame_msg[props[0]])
            else:
                return tuple([float(frame_msg.get(key)) for key in props])

        msg_time            = get_prop(["Frame"])
        if msg_time == 0: ## there is a weird incomplete message, I think it is a status message, that crashes the saver. it doesn't show up very often so it is hard to debug. 
            return
        side                = int(get_prop(["side"]))
        msg_total_force     = get_prop(["totalForce"])
        msg_cop             = get_prop(["cop1","cop2"])
        msg_ang             = get_prop(["ang1","ang2","ang3"])   
        msg_acc             = get_prop(["acc1","acc2","acc3"]) 
        msg_pres            = get_prop(["P%d"%sensor for sensor in range(1,17)]) # P1...P16
        return msg_time, side, msg_pres, msg_acc, msg_ang, msg_total_force, msg_cop 

    def __del__(self):
        #closes file
        if self.file:
            self.file.close()

class InsoleSrv:
    def __init__(self):
        self.recording = False
        self.savedict_list = []
        self.savefile_name = "/tmp/insole.txt"
   
    def init(self):
        #rospy.init_node("moticon_insoles", anonymous=True)
        self.publish_transforms = rospy.get_param("~publish_transforms", True)
        self.waiting = rospy.get_param("~wait_for_trigger", default=False)
        if self.waiting:
            rospy.logwarn("waiting for service trigger to start playback")
        else:
            rospy.logwarn("playback will start straightaway")

        self.ips = InsolePublishers()

        self.l_frame = rospy.get_param("~left_cop_reference_frame", default="map")
        self.r_frame = rospy.get_param("~right_cop_reference_frame", default="map")

        self.foot_length = rospy.get_param("~foot_length", default=1)
        self.foot_width = rospy.get_param("~foot_width", default=0.5)
        for i,side in enumerate(["left","right"]):
            self.ips.pressure[i] = rospy.Publisher(side+'/pressure', Common, queue_size=1)
            self.ips.force[i] = rospy.Publisher(side+'/force', Float32, queue_size=1)
            self.ips.time[i] = rospy.Publisher(side+'/time_diff', Float32, queue_size=1)
            self.ips.cop[i] = rospy.Publisher(side+'/cop', Common, queue_size=1) ## here I probably want a geometry Point or a Point[]
            self.ips.wrench[i] = rospy.Publisher(side+'/wrench', WrenchStamped, queue_size=1)
            self.ips.imu[i] = rospy.Publisher(side+'/imu_raw', Imu, queue_size=1)
            self.ips.insole[i] = rospy.Publisher(side+"/insole", InsoleSensorStamped, queue_size=1)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.execution_timer = rospy.Publisher("/insoles", Float32, queue_size=1)

        #New addition. We are going to publish the values with the delay that they actually have and rely on ROS to synchronize them
        #Good luck to us.

        self.estimated_delay = rospy.get_param("~estimated_delay", default=0) # in seconds, because SI.

        insole_rate = rospy.get_param("~insole_rate", default=100)
        node_freq = 2*insole_rate
        rospy.loginfo(f"Insole rate read freq set to {insole_rate}")
        rospy.loginfo(f"Node freq will be set to {node_freq}")
        self.rate = rospy.Rate(node_freq) ## if we read at 100 hertz from 2 sensors this should be enough?
        self.last_time = [None,None] ## left and right have different counters!
        self.this_time = [None,None]

        #recording service
        self.s = rospy.Service('~record', Empty, self.turn_on_recording)
        self.s1 = rospy.Service('~stop', Empty, self.turn_off_recording)
        self.s2 = rospy.Service('~save', Empty, self.save)
        self.s3 = rospy.Service('~setfilename', SetFileNameSrv, self.setfilename)
        self.s4 = rospy.Service('~clear', Empty, self.clear)
        self.s5 = rospy.Service('~start_playback', Empty, self.startplayback)

    def set_getter(self, getter):
        self.getter = getter

    def startplayback(self, req):
        self.waiting = False
        return EmptyResponse()

    def turn_on_recording(self, req):
        rospy.loginfo("Started recording")
        self.recording = True
        return EmptyResponse()

    def turn_off_recording(self, req):
        rospy.loginfo("Stopped recording")
        self.recording = False
        return EmptyResponse()

    def save(self, req):
        insole_data_save(self.savefile_name, self.savedict_list)
        rospy.loginfo("Recorded data saved in %s"%self.savefile_name)
        return EmptyResponse()

    def clear(self, req):
        self.savedict_list = []
        rospy.loginfo("Recorded data cache cleared.")
        return EmptyResponse()

    def setfilename(self, req):
        rospy.loginfo("Using directory for savedata: %s"%req.path)
        rospy.loginfo("Using filename for savedata: %s"%req.name)
        self.savefile_name = req.path + "/" + req.name + "_insole.txt"
        return SetFileNameSrvResponse()

    def run_server(self, ):

        while not rospy.is_shutdown(): ## maybe while ros ok
            rospy.loginfo_once("Will start listening")
            self.getter.start_listening()
            self.rate.sleep()
            try:
                while not rospy.is_shutdown() and self.getter.ok: ## we maybe want to rate limit this.
                    rospy.logdebug("Inner loop listening")
                    if self.waiting:
                        self.rate.sleep()
                        continue
                    tic = time.perf_counter()

                    try:
                        msg_time, side, msg_press, msg_acc, msg_ang, msg_total_force, msg_cop = self.getter.get_data()
                    except StopIteration:
                        break
                    except Exception as e:
                        print(e)
                        print("something went wrong when getting data!")
                        continue
                    #create dict we want to save later:
                    if self.recording:
                        #rospy.loginfo("REC\r")
                        try:
                            self.savedict_list.append(extract_insole_data(msg))

                        except Exception as exc:
                            rospy.logerr("could not create savedict. data for this frame will not be saved.%s"%exc)

                    # Filter because insole is noisy.
                    #TODO: actually the saved data would allow us to space the samples appropriately in time and also filter them. To do it rt we would need a buffer, so consider this.


                    # Publish these guys
                    h = Header()
                    time_stamp = rospy.Time.now() - rospy.Duration(self.estimated_delay)
                    h.stamp = time_stamp

                    msg_insole_msg = InsoleSensorStamped()

                    ## now I need to publish it to the right side. 
                    ## maybe this is wrong and I need to publish them both at the same time, but since I receive a message which is from either one side or the other, than the other side's info would be zero, so I didn't solve anything by doing this, I just pushed the problem further. At some point I need to remember which side is doing what. I can't rely on tf for this, so maybe I need to remember the latest values, update them here and publish both at the same time?


                    t = TransformStamped()
                    if side: ## or the other way around, needs checking
                        h.frame_id = "right"
                        t.child_frame_id = "right"
                        x_axis_direction = 1
                        t.header.frame_id = self.r_frame

                    else:
                        h.frame_id = "left"
                        t.child_frame_id = "left"
                        x_axis_direction = -1
                        t.header.frame_id = self.l_frame
                    msg_insole_msg.header = h

                    pressure = 0
                    force = 0
                    cop = (0,0)
                    if not msg_time:
                        #rospy.logwarn("no time in message!")
                        pass
                    else:
                        self.this_time[side] = msg_time
                        if self.this_time[side] and self.last_time[side]:
                            timediff = self.this_time[side] - self.last_time[side]   
                            rospy.logdebug("time counter %d, time difference %d"%(self.this_time[side], timediff))
                            tmsg =  Float32(timediff)
                            self.ips.time[side].publish(tmsg)
                            msg_insole_msg.time = tmsg
                        self.last_time = deepcopy(self.this_time)
                    if not msg_total_force:
                        #rospy.logwarn("no total_force. not publishing force or wrench topics")
                        pass
                    else:
                        fmsg = Float32(msg_total_force)
                        try:
                            self.ips.force[side].publish(fmsg)
                        except Exception as e:
                            rospy.logerr("total force looks like this: %s and I can't publish it!! %s"%(msg_total_force, e))
                        force = Vector3(y=msg_total_force)
                        wren = Wrench(force=force) #force, torque
                        wmsg = WrenchStamped(h,wren)
                        msg_insole_msg.force = fmsg 
                        msg_insole_msg.wrench = wren
                        self.ips.wrench[side].publish(wmsg)
                    ## we also want to send a tf for the cop


                    if not msg_cop:
                        #rospy.logwarn_once("no cop. not publishing ...")
                        pass
                    else:
                        #print("Issuign transforms")
                        msg_insole_msg.cop.data = msg_cop
                        cmsg = Common(h, msg_cop)
                        self.ips.cop[side].publish(cmsg)
                        t.header.stamp = time_stamp
                        t.transform.translation.x = self.foot_width/2*(msg_cop[1])*x_axis_direction ### need to check these because I am rotating them with the static transform afterwards...
                        t.transform.translation.y = self.foot_length*(msg_cop[0] + 0.5) 
                        t.transform.translation.z = 0
                        #t.transform.rotation.x = 0
                        #t.transform.rotation.y = 0.707
                        #t.transform.rotation.z = 0.707
                        #t.transform.rotation.w = 0
                        t.transform.rotation = OpenSimTf.rotation
                        msg_insole_msg.ts = t
                        if self.publish_transforms:
                            self.broadcaster.sendTransform(t)
            
                    if not msg_press:
                        #rospy.logwarn_once("no pressure data. not publishing")
                        pass
                    else:
                        pmsg = Common(h, msg_press)
                        self.ips.pressure[side].publish(pmsg)
                        msg_insole_msg.pressure.data = msg_press
                    if not msg_ang or not msg_acc:
                        #rospy.logwarn_once("no angular or acceleration data. cannot publishing imu_msg")
                        pass
                    else:
                        imsg = convert_to_imu(h, msg_ang, msg_acc)
                        self.ips.imu[side].publish(imsg)
                        msg_insole_msg.imu = imsg

                    self.ips.insole[side].publish(msg_insole_msg)
                    self.rate.sleep()
                    toc = time.perf_counter()
                    self.execution_timer.publish((toc-tic)*1000)
                    #rospy.loginfo(f"time it took to run over loop once {(toc - tic)*1000:0.4f} ms")
                #raise rospy.ROSInterruptException("This is fine. It's the way to close this otherwise it will run forever.")
                break
            finally:
                del self.getter
