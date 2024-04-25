#!/usr/bin/env python3

print(f"Loaded {__file__}.py")

import os
import rospy
import csv

from insoles_common.insole_data_getter import InsoleDataGetter
from insoles_common.better_rate import BetterRate

class InsoleDataFromFile(InsoleDataGetter):
    """Data from file class"""
    def __init__(self, filename = ""):
        self.filename = filename
        self.file = None
        self.reader = None
        self.data = []
        self.insole_rate = rospy.get_param("~insole_rate", default=100)
        node_freq = 2*self.insole_rate
        rospy.loginfo(f"Insole rate read freq set to {self.insole_rate}")
        rospy.loginfo(f"Node freq will be set to {node_freq}")
        self.rate = BetterRate(node_freq) ## if we read at 100 hertz from 2 sensors this should be enough?
        self.sensors =["P%d"%sensor for sensor in range(1,17)] 
    
        ###

        self.use_synchronization_event = rospy.get_param("~use_synchronization_event", default=False)

    def set_start_time_from_sync_event(self):
        """
            This is a bit tricky. 

            I will calculate what would have been the starting frame for each side based on the start_time and the
            known time in the past where each insole was known to have been synchronized. 


        """
        rsecs = rospy.get_param("~rsecs")
        rnsecs = rospy.get_param("~rnsecs")
        rto = rospy.get_param("~rto")
        
        lsecs = rospy.get_param("~lsecs")
        lnsecs = rospy.get_param("~lnsecs")
        lto = rospy.get_param("~lto")

        insole_sync_secs = [lsecs, rsecs]
        insole_sync_nsecs = [lnsecs, rnsecs]
        insole_sync_to = [lto, rto]

        for i in [0,1]:
            this_side_sync_time = rospy.Time()
            this_side_sync_time.set(insole_sync_secs[i],insole_sync_nsecs[i])
            ## caveat, the start_time should be larger than the sync time
            start_time_time = rospy.Time.from_sec(self.start_time)
           
           ## this is a duration
            elapsed_frames_this_side = (start_time_time - this_side_sync_time).to_sec()*1000
            self.start_frame[i] = insole_sync_to[i] + int(elapsed_frames_this_side)


    def set_start_time(self):
        """ Default start_time is zero """
        self.start_time = rospy.get_param("~start_time", default=0)
        
    def start_listening(self):
        #maybe opens file and we have a getline thing going
        print(self.filename)
        self.file = open(self.filename, "r")
        ## maybe file operations are too slow to be executed inside the loop. let's test this out
        reader = csv.DictReader(self.file, delimiter=" ")
        for row in reader:
            self.data.append(row)
        self.set_start_time() ## this is a bit different from live and file, but I think it is okay.
        #print(self.data)
        ### default, if there isnt any problem, this should work.

        if self.use_synchronization_event:
            rospy.logwarn("using EXT sync event from params")
            self.set_start_time_from_sync_event()
        else:
            rospy.loginfo("Using first frame as sync event")
            for row in self.data:
                if row["side"] == "0" and not self.start_frame[0]:
                    self.start_frame[0] = int(row["Frame"])
                if row["side"] == "1" and not self.start_frame[1]:
                    self.start_frame[1] = int(row["Frame"])
                if self.start_frame[0] and self.start_frame[1]:
                    break
        print(f"start_frame from insoles: {self.start_frame}")
        self.reader = iter(self.data)
        #print(next(self.reader))

    @property
    def ok(self):
        #maybe checks if there is still things to be read.
        if self.file:
            return True
        else:
            return False

    def get_data(self):
        if not self.start_time:
            self.set_start_time()
        if rospy.Time.now() < rospy.Time.from_sec(self.start_time):
            return

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

        msg_time            = int(get_prop(["Frame"]))
        if msg_time == 0: ## there is a weird incomplete message, I think it is a status message, that crashes the saver. it doesn't show up very often so it is hard to debug. 
            return
        side                = int(get_prop(["side"]))
        msg_total_force     = get_prop(["totalForce"])
        msg_cop             = get_prop(["cop1","cop2"])
        msg_ang             = get_prop(["ang1","ang2","ang3"])   
        msg_acc             = get_prop(["acc1","acc2","acc3"]) 
        msg_pres            = get_prop(self.sensors) # P1...P16
        self.rate.sleep()
        return None, msg_time, side, msg_pres, msg_acc, msg_ang, msg_total_force, msg_cop 

    def close(self):
        #closes file
        if self.file:
            self.file.close()
            self.filename = None # prevents for running many times  
            rospy.loginfo("closed successfully.")

