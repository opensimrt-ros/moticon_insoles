#!/usr/bin/env python3
import socket
import moticon_insoles
import rospy
import tf2_ros
from std_msgs.msg import Header
from opensimrt_msgs.msg import Common 
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped, TransformStamped
from colorama import Fore

class InsolePublishers():
    def __init__(self):
        self.pressure = ["",""]
        self.force = ["",""]
        self.cop = ["",""]
        self.wrench = ["",""]

def run_server(server_name = "", port = 9999):
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

    for i,side in enumerate(["left","right"]):
        ips.pressure[i] = rospy.Publisher(side+'_pressure', Common, queue_size=1)
        ips.force[i] = rospy.Publisher(side+'_force', Common, queue_size=1)
        ips.cop[i] = rospy.Publisher(side+'_cop', Common, queue_size=1) ## here I probably want a geometry Point or a Point[]
        ips.wrench[i] = rospy.Publisher(side+'_wrench', WrenchStamped, queue_size=1) ## here I probably want a geometry Point or a Point[]

    broadcaster = tf2_ros.TransformBroadcaster()

    
    rate = rospy.Rate(1000) ## if we read at 100 hertz from 2 sensors this should be enough?
    while not rospy.is_shutdown(): ## maybe while ros ok
        rospy.loginfo('Waiting for a connection...')
        connection, client_address = sock.accept()
        try:
            rospy.logdebug('client connected: {}'.format(client_address))
            while not rospy.is_shutdown(): ## we maybe want to rate limit this.
                try:
                    msg_buf = moticon_insoles.get_message(connection)
                except moticon_insoles.ConnectionClosed as e:
                    rospy.logerr(e)
                    break

                msg = moticon_insoles.proto_s.MoticonMessage() ## maybe this can be outside the loop
                msg.ParseFromString(msg_buf)

                # Now handle the message
                color = ""
                if msg.data_message.side:
                    color =Fore.CYAN # RIGHT SIDE ## consider adding for debug
                else:
                    color =Fore.LIGHTCYAN_EX
                rospy.logdebug(color+str(msg)) ## consider adding for debug

                #rospy.logdebug(color+dir(msg.data_message))
                rospy.logdebug(color+str(msg.data_message.pressure))
                rospy.logdebug(color+str(msg.data_message.total_force)) ## to display in Rviz we need to use a WrenchStamped
                rospy.loginfo(color+str(msg.data_message.cop)) ## maybe this is a geometry/Point

                ### this will be relevant for the sensors/imu raw publisher. we also need to make sure it is using SI and they are the same as ROS's definitions
                ## I will also need this if we have an inclined plane!!!
                rospy.logdebug(color+str(msg.data_message.angular))
                rospy.logdebug(color+str(msg.data_message.acceleration))

                #rospy.logdebug(color+str(msg.data_message.time) ## not sure if I need this guy
                #rospy.logdebug(color+str(msg.data_message.temperature) ## in our device this is always zero
                rospy.logdebug(color+str(msg.data_message.service_id)) ## not sure what this is either
                rospy.logdebug(color+str(msg.data_message.side))


                
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
                    offset = 0.3
                else:
                    h.frame_id = "left"
                    t.child_frame_id = "left"
                    offset = -0.3
                
                pmsg = Common(h, msg.data_message.pressure)
                fmsg = Common(h, msg.data_message.total_force)
                cmsg = Common(h, msg.data_message.cop)

                force = Vector3(y=msg.data_message.total_force)
                wren = Wrench(force=force) #force, torque
                wmsg = WrenchStamped(h,wren)
                
                ips.pressure[side].publish(pmsg)
                ips.force[side].publish(fmsg)
                ips.cop[side].publish(cmsg)
                ips.wrench[side].publish(wmsg)
                ## we also want to send a tf for the cop

                t.header.frame_id = "map" # TODO: should be a param
                if len(msg.data_message.cop)>0:
                    t.header.stamp = time_stamp
                    t.transform.translation.x = msg.data_message.cop [1] + offset### need to check these because I am rotating them with the static transform afterwards...
                    t.transform.translation.y = msg.data_message.cop [0]
                    t.transform.translation.z = 0
                    t.transform.rotation.x = 0
                    t.transform.rotation.y = 0.707
                    t.transform.rotation.z = 0.707
                    t.transform.rotation.w = 0
                    broadcaster.sendTransform(t)
                else:
                    rospy.logerr_throttle(1,"cannot read COP from insole. Are you sure you are sending this?")
                rate.sleep()
            raise rospy.ROSInterruptException()
        finally:
            connection.close()

#moticon_insoles.run_server()
run_server()

