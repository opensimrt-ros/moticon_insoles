#!/usr/bin/env python3
from insoles_common.insole_srv import InsoleSrv
from insoles_common.insole_data_from_nothing import InsoleDataFromNothing
from insoles_common import *
import rospy

insrv = InsoleSrv()
rospy.init_node("insole_from_nothing")
getter = InsoleDataFromNothing()
insrv.set_getter(getter)
insrv.init()
insrv.run_server()
rospy.spin()

