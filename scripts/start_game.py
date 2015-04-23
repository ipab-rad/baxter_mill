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
from baxter_mill.srv import *

class Game(object):

    def __init__(self, limb):
        self.bc = BaxterController(limb)
        self.bc.send_image(self.bc._cheeky_face_path)

    def test(self, limb):
        print "STARTING TEST!"
        self.bc.pick("p1")
        self.bc.release("a1")
        print "TEST FINISHED!"

    def rec_command(self, req):
        res = BaxterCommandResponse()
        print "Received service!"
        print req.pos1 + " " + req.pos2
        acceptables = self.bc.acceptable['left'] + self.bc.acceptable['positions']
        if req.pos1 == 'neutral' or req.pos2 == 'neutral':
            self.bc.go_neutral()
            print "Going neutral"
        elif req.pos1 in acceptables and req.pos2 in acceptables:
            self.bc.pick(req.pos1)
            self.bc.release(req.pos2)
            print "Going from " + req.pos1 + " to " + req.pos2
        else:
            print "Aborting, no acceptable targets set"
            res.res = False
            return True
        res.res = True
        return True

    def command_server(self):
        #rospy.init_node('command_server')
        s = rospy.Service('baxter_command', BaxterCommand, self.rec_command)
        print "Command server ready"

def main():
    limb = "left"
    rospy.init_node('baxter_mill_%s' % (limb))
    g = Game(limb)
    rospy.sleep(1.0)
    #test(limb)
    g.command_server()
    while(1):
        rospy.sleep(0.1)

if __name__ == "__main__":
    main()
