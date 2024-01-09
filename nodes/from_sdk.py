#!/usr/bin/env python3
from insoles_common.insole_srv import InsoleSrv
from insoles_common.insole_data_from_socket import InsoleDataFromSocket
import rospy 

insrv = InsoleSrv()
rospy.init_node("insole_rt")
getter = InsoleDataFromSocket()
insrv.set_getter(getter)
insrv.init()
insrv.run_server()
rospy.spin()

