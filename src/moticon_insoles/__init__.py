#from __future__ import absolute_import 
print("Loading Moticon Endpoint SDK")

import sys,os
#print(__file__)
#print(os.path.dirname(os.path.realpath(__file__)))
#print(sys.path[0])
#sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)) + '/sdk/python_example')
## this has to do with something from ROS not being configured properly. It is a hack to make it work
sys.path.insert(0, sys.path[0] + '/moticon_insoles/sdk/python_example')
#print(sys.path)
from .sdk.python_example.endpoint import *
#import sdk
#import test
