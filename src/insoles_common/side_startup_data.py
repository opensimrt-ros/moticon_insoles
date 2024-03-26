#!/usr/bin/env python3
print(f"Loaded {__file__}")
import rospy

class SideStartupData():
    def __init__(self):
        self.data = []
        self.start_time = ["",""]
        self.rate = ["",""]
        self.service_configs =["",""]
    def store_startup_data(self, msg):
        rospy.logwarn("Got this message with a strange time_stamp. Assuming it is startup data.\nDoing nothing with it now. Just saving. :\n%s"%msg)
        self.data.append(msg)
        try: 
            if msg.data_message.HasField("side") and msg.data_message.HasField("service_config"): 
                side = int(msg.data_message.side)
                self.service_config[side] = msg.data_message.service_config 
                self.start_time[side] = msg.service_config.service_start_time 
                self.rate[side] =  msg.service_configs.rate
        except Exception as e:
            rospy.logwarn(e)

