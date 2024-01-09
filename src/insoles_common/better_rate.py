#!/usr/bin/env python3
print(f"Loaded {__file__}")

import rospy
from multiprocessing import Lock

class BetterRate:
    """BetterRate
        
        This shouldnt be necessary, but normal rospy.Rate sometimes misbehaves with simulated clock, so we need this to better keep track of running times. It sacrifices loop times for overall runtime.
    """
    def __init__(self,frequency):
        self.frequency_slow = float(frequency)
        self.frequency_fast = 1/(1/frequency -0.001) ## This number should be artificial clock time step
        self.freq_fast_rate = rospy.Rate(self.frequency_fast)
        self.freq_slow_rate = rospy.Rate(self.frequency_slow)
        self.mutex = Lock() # I dont really need this, but it doesnt hurt
        with self.mutex:
            self.ticks = 0
    def sleep(self):
        if self.ticks == 0:
            ## uncomfortable not to be able to use the init for this, but time would be set to zero.
            self.initial_time = rospy.Time.now()
            self.ticks = 1
            self.freq_slow_rate.sleep()
            self.freq_fast_rate.last_time = self.freq_slow_rate.last_time
            return 
        self.ticks +=1
        time_now = rospy.Time.now()
        with self.mutex:
            local_initial_time = self.initial_time
        running_time = time_now - local_initial_time
        ## measure if we are running late 
        running_frequency = self.ticks/running_time.to_sec() 
        #rospy.loginfo(f"\ninitial_time: {self.initial_time}\nlocal_initial_time:{local_initial_time}\ntime_now= {time_now}\nrunning_time:{running_time}\nrunning_frequency: {running_frequency} ")
        if float(running_frequency) < self.frequency_slow:
            self.freq_fast_rate.sleep()
            rospy.logwarn("runnning late.")
            # but the other frequency didnt run, se we need to reset their last time
            self.freq_slow_rate.last_time = self.freq_fast_rate.last_time
        else:
            self.freq_slow_rate.sleep()
            # but the other frequency didnt run, se we need to reset their last time
            self.freq_fast_rate.last_time = self.freq_slow_rate.last_time

