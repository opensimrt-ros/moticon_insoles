#!/usr/bin/env python3
print(f"Loaded {__file__}")
import moticon_insoles
import rospy
import socket

from insoles_common.insole_data_getter import InsoleDataGetter
from insoles_common.side_startup_data import SideStartupData 

class InsoleDataFromSocket(InsoleDataGetter):
    """Sensor reader class"""
    def __init__(self, server_name = "", port = 9999):
        self.connection = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (server_name, port)
        rospy.loginfo('Starting Moticon Insole server up on {} port {}'.format(server_name, port))
        self.sock.bind(server_address)
        self.sock.listen(1)
        self.connection_ok = True
        self.insole_startup_data = [SideStartupData(), SideStartupData()]
    def set_start_time(self):
        """ Default start_time is the time of the first frame actually. I need to make sure this is close, otherwise the calculations for frame time transformations will be incorrect. """
        self.start_time = rospy.Time.now().to_sec()

    @property
    def ok(self):
        return self.connection_ok

    def start_listening(self):
        if self.ok:
            rospy.loginfo('Waiting for a connection...')

            self.connection, client_address = self.sock.accept()
            rospy.loginfo('Connection created.')
            rospy.logdebug('client connected: {}'.format(client_address))
        else:
            rospy.logfatal("I need to recreate the socket, maybe with new startup data. I haven't checked if it works yet, do not use")
            raise(Exception)

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
            ## this is actually a service start message with tons of interesting information that might help synchronize things better!
            self.parse_startup_message(msg)
            return ## prevents start_time being set to zero when we have startup messages without timestamps.
        if not self.start_time:
            self.set_start_time()
        if not self.start_frame[side]:
            self.start_frame[side] = msg_time
        msg_total_force = msg.data_message.total_force
        msg_cop = msg.data_message.cop
        msg_ang = msg.data_message.angular
        msg_acc = msg.data_message.acceleration
        msg_pres = msg.data_message.pressure
        return msg_time, side, msg_pres, msg_acc, msg_ang, msg_total_force, msg_cop 

    def parse_startup_message(self, msg):
        if msg.data_message.side == 1:
            self.insole_startup_data[1].store_startup_data(msg)
        elif msg.data_message.side == 0:
            self.insole_startup_data[0].store_startup_data(msg)

    def close(self):
        if self.connection:
            self.connection.close()
            self.connection_ok = False
            rospy.loginfo("disconnected successfully.")
