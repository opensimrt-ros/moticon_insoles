#!/usr/bin/env python3
print(f"Loaded {__file__}")
import rospy

class SideStartupData():
    def __init__(self):
        self.data = None
        self.start_time = None
        self.rate = None
        self.service_configs = None
    def store_startup_data(self, msg):
        rospy.logwarn("Got this message with a strange time_stamp. Assuming it is startup data.\nDoing nothing with it now. Just saving. :\n%s"%msg)
        self.data = msg
        try: 
            self.service_configs = msg.data_message["service_configs"] 
            self.start_time = msg["service_configs"]["service_start_time"] 
            self.rate =  msg["service_configs"]["rate"]
        except Exception as e:
            rospy.logwarn(e)

