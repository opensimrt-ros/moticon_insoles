#!/usr/bin/env python3

print(f"Loaded {__file__}")

import os
from pathlib import Path

from sensor_msgs.msg import Imu
from math import pi
import numpy as np

from geometry_msgs.msg import Transform
GRAVITY = 9.80665

OpenSimTf = Transform()
OpenSimTf.rotation.x = 0
OpenSimTf.rotation.y = 0.707
OpenSimTf.rotation.z = 0.707
OpenSimTf.rotation.w = 0

class InsolePublishers():
    def __init__(self):
        self.imu = ["",""]
        self.insole = ["",""]
        self.delay_publisher = [None, None]

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

def extract_insole_data(msg_insole, time=0):
    """Extract only the pressure and acc info from the large streaming data"""

    saving_data = [msg_insole.data_message.time, msg_insole.data_message.side,\
            *msg_insole.data_message.pressure,\
            *np.around(msg_insole.data_message.acceleration, decimals=3),\
            *np.around(msg_insole.data_message.angular, decimals=3),\
            msg_insole.data_message.total_force,\
            *np.around(msg_insole.data_message.cop, decimals=5),time]

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
            'totalForce', 'cop1', 'cop2','ArrivalTime']

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

