#!/usr/bin/env python3
print(f"Loaded {__file__}")

import rospy
from std_msgs.msg import Header, Float32
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from std_srvs.srv import Empty, EmptyResponse
from opensimrt_msgs.srv import SetFileNameSrv, SetFileNameSrvResponse
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped, TransformStamped, Transform
from insole_msgs.msg import InsoleSensorStamped
from sensor_msgs.msg import Imu
from copy import deepcopy
from multiprocessing import Lock

#import time
from insoles_common.utils import *

class InsoleSrv:
    def __init__(self):
        self.recording = False
        self.savedict_list = []
        self.savefile_name = "/tmp/insole.txt"
        self.mutex = Lock()
        self.debug_flip = 1

    def init(self):
        rospy.logwarn("TODO: using informed insole_rate as param! this is less than ideal and can lead to errors. read it from the insole setup message instead ")
        self.insole_rate = rospy.get_param("~insole_rate", default=100)
        with self.mutex:
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
        self.grf_origin_z_offset = rospy.get_param("~grf_origin_z_offset", default=0.0)
        for i,side in enumerate(["left","right"]):
            self.ips.imu[i] = rospy.Publisher(side+'/imu_raw', Imu, queue_size=1)
            self.ips.insole[i] = rospy.Publisher(side+"/insole", InsoleSensorStamped, queue_size=1)
            # let's publish the measured delay
            self.ips.delay_publisher[i] = rospy.Publisher(side+'/delay',Float32, queue_size=1)

        self.diagspub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.last_time = [None,None] ## left and right have different counters!
        self.this_time = [None,None]

        self.s = rospy.Service('~start_recording', Empty, self.turn_on_recording)
        self.s1 = rospy.Service('~stop_recording', Empty, self.turn_off_recording)
        ## it isnt a sto, but we can use the same names as the saver_node then everything gets easier
        self.s2 = rospy.Service('~write_sto', Empty, self.save)
        self.s3 = rospy.Service('~set_name_and_path', SetFileNameSrv, self.setfilename)
        self.s4 = rospy.Service('~clear', Empty, self.clear)
        self.s5 = rospy.Service('~start_playback', Empty, self.startplayback)

        
    def set_getter(self, getter):
        self.getter = getter

    def startplayback(self, req):
        with self.mutex:
            self.waiting = False
        return EmptyResponse()

    def turn_on_recording(self, req):
        with self.mutex:
            rospy.loginfo("Started recording")
            self.recording = True
        return EmptyResponse()

    def turn_off_recording(self, req):
        with self.mutex:
            rospy.loginfo("Stopped recording")
            self.recording = False
        return EmptyResponse()

    def save(self, req):
        with self.mutex:
            insole_data_save(self.savefile_name, self.savedict_list)
            rospy.loginfo("Recorded data saved in %s"%self.savefile_name)
        return EmptyResponse()

    def clear(self, req):
        with self.mutex:
            self.savedict_list = []
            rospy.loginfo("Recorded data cache cleared.")
        return EmptyResponse()

    def setfilename(self, req):
        with self.mutex:
            rospy.loginfo("Using directory for savedata: %s"%req.path)
            rospy.loginfo("Using filename for savedata: %s"%req.name)
            self.savefile_name = req.path + "/" + req.name + "_insole.txt"
        return SetFileNameSrvResponse()

    def get_timestamp(self, frame_count, side):
        return rospy.Time.from_sec(self.get_insole_walltime(frame_count,side)+ self.getter.start_time)
    
    def get_insole_walltime(self, frame_count, side):
        framestart = self.getter.start_frame[side]
        if False:
            if side:
                rospy.logwarn(f"\t\t\t{framestart}") # logger_name="frame_start"
            else:
                rospy.logwarn(f"\t{framestart}") 

        return (frame_count - self.getter.start_frame[side])/1000.

    def get_ros_walltime(self):
        return rospy.Time.now().to_sec() - self.getter.start_time
    def get_frame_delay(self, frame_count,side):
        ## compare ros walltime with insole
        ros_wall_time = self.get_ros_walltime()
        #rospy.loginfo(f"ros ros_wall_time {ros_wall_time}")
        insole_wall_time = self.get_insole_walltime(frame_count,side)

        if False:
            if side:
                rospy.loginfo(f"\t\t\t{insole_wall_time}") ## use the ,logger_name="insole_wall_time"
            else:
                rospy.loginfo(f"\t{insole_wall_time}") 

        return ros_wall_time - insole_wall_time
    
    def loop_once(self):
        rospy.logdebug("Inner loop listening")
        diag_msg_array = DiagnosticArray()
        with self.mutex:
            if self.waiting:
                rospy.logwarn_once("waiting...")
                return

        response = self.getter.get_data()
        if response:
            if len(response) == 8:
                msg, msg_time, side, msg_press, msg_acc, msg_ang, msg_total_force, msg_cop = response
                self.started = True
            else:
                ## we are receiving one of those status messages, i think. 
                ## I should probably do something with this information, however currently I will just log it
                rospy.logwarn(f"NOT IMPLEMENTED: Received status response from insole, but I don't know what to do with this information\n{response}")
        else:
            with self.mutex:
                if self.started:
                    ##I started but got a none, so this means that I stopped?
                    self.started = False
                    rospy.logwarn("I think I have stopped. last published time: %s"%self.time_stamp)
                    raise(StopIteration)
            return
        with self.mutex:
            if self.recording:
                try:
                    self.savedict_list.append(extract_insole_data(msg))

                except Exception as exc:
                    rospy.logerr("could not create savedict. data for this frame will not be saved.%s"%exc)

        h = Header()
        self.time_stamp = self.get_timestamp(msg_time,side)
        h.stamp = self.time_stamp
        rospy.logwarn_once("start_time that is actually used for header: %s"%self.time_stamp)

        msg_insole_msg = InsoleSensorStamped()

        #
        delay_msg = Float32()
        
        diags_time = DiagnosticStatus()
        diags_time.name = "insole_%s"%('right' if side else 'left')
        diags_time.hardware_id = "some_hardware_id_%s"%('r' if side else 'l')
        time_delay_for_frame = self.get_frame_delay(msg_time,side)
        diags_time.message = f"Frame delay is {time_delay_for_frame}s."
        if (time_delay_for_frame>1): # this time should be in seconds
            diags_time.level = diags_time.STALE
        else:
            diags_time.level = diags_time.OK
        diag_msg_array.status.append(diags_time)
        delay_msg.data = time_delay_for_frame
        self.ips.delay_publisher[side].publish(delay_msg)


        t = TransformStamped() # i dont need the time, but i need to have the child_frame_id

        if side: ## or the other way around, needs checking
            h.frame_id = self.r_frame
            t.child_frame_id = "right_cop"
            x_axis_direction = 1

        else:
            h.frame_id = self.l_frame
            t.child_frame_id = "left_cop"
            x_axis_direction = -1

        t.header = h
        msg_insole_msg.header = h

        if msg_time:
            self.this_time[side] = msg_time
            if self.this_time[side] and self.last_time[side]:
                timediff = self.this_time[side] - self.last_time[side]   
                #in the past maybe I wanted to know this, but now its meaning will change. I don't remember if I am using this anywhere other than graphing the jitter, so I will hijack this to a more useful case. if you want a jitter, change the msg def.
                #tmsg =  Float32(timediff)
                tmsg = Float32(msg_time)                
                msg_insole_msg.time = tmsg
            self.last_time = deepcopy(self.this_time)
        if msg_total_force:
            fmsg = Float32(msg_total_force)
            force = Vector3(y=msg_total_force)
            wren = Wrench(force=force) #force, torque)
            msg_insole_msg.force = fmsg 
            msg_insole_msg.wrench = wren

        if msg_cop:
            msg_insole_msg.cop.data = msg_cop
            t.transform.translation.x = -self.foot_width*(msg_cop[1])*x_axis_direction ### need to check these because I am rotating them with the static transform afterwards...
            t.transform.translation.y = self.foot_length*(msg_cop[0]) 
            t.transform.translation.z = self.grf_origin_z_offset

            if False:
                #t.transform.translation.x = 0.4*x_axis_direction
                t.transform.translation.x = self.foot_width/2*x_axis_direction #*self.debug_flip
                t.transform.translation.y = -self.foot_length/2 
                t.transform.translation.z = 0 #self.grf_origin_z_offset
            t.transform.rotation = OpenSimTf.rotation
            msg_insole_msg.ts = t

        if msg_press:
            msg_insole_msg.pressure.data = msg_press
        if msg_ang and msg_acc:
            imsg = convert_to_imu(h, msg_ang, msg_acc)
            msg_insole_msg.imu = imsg

        self.ips.insole[side].publish(msg_insole_msg)
        self.last_time_stamp = deepcopy(self.time_stamp)
        self.debug_flip = -self.debug_flip
        self.diagspub.publish(diag_msg_array)

    def run_server(self, ):

        while not rospy.is_shutdown(): ## maybe while ros ok
            rospy.loginfo_once("Will start listening")
            self.getter.start_listening()
            with self.mutex:
                self.started = False
            self.time_stamp = None
            self.last_time_stamp = None
            if not self.getter.ok:
                rospy.sleep(0.001) 
                continue
            else:
                rospy.logwarn("################### start_time from getter: %s"%self.getter.start_time)

            while not rospy.is_shutdown() and self.getter.ok: ## we maybe want to rate limit this.
                try:
                    #tic = time.time()
                    self.loop_once()
                    #toc1 = time.time()
                    
                    ## interesting caveat of using a simulated clock with python in ros1 is that the rates are slower, at least in my machine, than what you would expect. while in rt mode, you have sometimes faster loops that probably correct the overall rate, in simulated clock this is not the case and the rate might be slower than you would expect!
                    #toc2 = time.time()
                    #print("%f loop time\n\tloop+wait time:%f"%((toc1-tic,toc2-tic)))
                except StopIteration:
                    rospy.logwarn_once("I got a StopIteration exception, so I must have stopped. last published time: %s"%self.time_stamp)
                    self.getter.close()
                    break
                except KeyboardInterrupt:
                    rospy.loginfo("Goodbye!")
                    self.getter.close()
                    sys.exit(0)
