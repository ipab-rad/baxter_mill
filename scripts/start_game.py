#!/usr/bin/python

import sys
import random
import rospkg
import rospy
from std_msgs.msg import (
    String,
)
from sensor_msgs.msg import (
    Image,
)

import baxter_interface
from baxter_mill import BaxterController

def test(limb):
    bc = BaxterController(limb)
    print "STARTING TEST!"
    bc.pick("p1")
    bc.release("a1")
    print "TEST FINISHED!"

def main():
    limb = "left"
    rospy.init_node('baxter_mill_%s' % (limb))
    rospy.sleep(1.0)
    test(limb)

if __name__ == "__main__":
    main()
