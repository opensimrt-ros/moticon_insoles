#!/usr/bin/env python
PKG = 'moticon_insoles'

import sys
import unittest
import tf
import rospy
class TestTF(unittest.TestCase):
    def __init__(self,*args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("test_tf")
        self.t = tf.TransformListener()
        # scrape rosparam
        self.tfs = []
        params = rospy.get_param('~tfs', [])
        self.reference_frame = rospy.get_param("~reference", "map")
        for param in params:
            if 'name' not in param:
                self.fail("'name' field in rosparam is required but not specified.")
            tfs = {'timeout': 10, 'negative': False}
            tfs.update(param)
            self.tfs.append(tfs)
        # check if there is at least one topic
        if not self.tfs:
            self.fail('No tfs is specified in rosparam.')
    
    def test_tf(self): # only functions with 'test_'-prefix will be run!
        rate =rospy.Rate(10)
        for i in range(40):
            #the tf system takes some time to start. it should be made explicit that this is what we are doing, but this works and I dont want to fix it. 
            print(self.t.allFramesAsString())
            rate.sleep()
        trans = None
        rot = None
        try:
            for a_frame in self.tfs:
                (trans, rot ) = self.t.lookupTransform(a_frame["name"], self.reference_frame, rospy.Time(0))
                self.assertIsNotNone(trans)
                self.assertIsNotNone(rot)
        except tf.LookupException as e:
            self.fail("cannot get tf %s"%e)
        except tf.ConnectivityException as e:
            self.fail("cannot get tf %s"%e)
        except tf.ExtrapolationException as e:
            self.fail("cannot get tf %s"%e)
        #self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_tf_node', TestTF)
