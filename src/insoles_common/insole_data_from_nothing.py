#!/usr/bin/env python3

print(f"Loaded {__file__}.py")

import os
import rospy
import csv

from insoles_common.insole_data_getter import InsoleDataGetter
from insoles_common.better_rate import BetterRate

class InsoleDataFromNothing(InsoleDataGetter):
    """Data from file class"""
    def __init__(self):
        self.insole_rate = rospy.get_param("~insole_rate", default=100)
        node_freq = 2*self.insole_rate
        rospy.loginfo(f"Insole rate read freq set to {self.insole_rate}")
        rospy.loginfo(f"Node freq will be set to {node_freq}")
        self.rate = BetterRate(node_freq) ## if we read at 100 hertz from 2 sensors this should be enough?
        self.sensors =["P%d"%sensor for sensor in range(1,17)] 
        ###
        self.initialized = False
        self.i=0
    
    def battery_level(self, side):
        return 100

    def set_start_time(self):
        self.start_time = 100000

    def start_listening(self):
        if self.initialized:
            return
        self.set_start_time() ## this is a bit different from live and file, but I think it is okay.

        self.start_frame = [0,0]
        print(f"start_frame from insoles: {self.start_frame}")
        self.initialized = True

    @property
    def ok(self):
        return True

    def get_data(self):
        if not self.start_time:
            self.set_start_time()
        
        self.i+=1
        
        frame_msg = {"Frame":self.start_frame[self.i%2]+self.i*5,
                    "side":self.i%2,
                    "acc1":0,
                    "acc2":0,
                    "acc3":0,
                    "ang1":0,
                    "ang2":0,
                    "ang3":0,
                    "totalForce":300+self.i%50,
                    "cop1":0,
                    "cop2":0,
                    }
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
        pass
