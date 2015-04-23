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
import cv
import cv_bridge

class BaxterController(object):

    def __init__(self, limb):
        self._rp = rospkg.RosPack()
        self._config_path = self._rp.get_path('baxter_mill') + '/config/'
        self._config_file_path = self._config_path + limb + '_positions.config'
        self._images_path = self._rp.get_path('learn_play') + '/share/images/'
        self._good_face_path = self._images_path + "good_face.jpg"
        self._angry_face_path = self._images_path + "angry_face.jpg"
        self._cheeky_face_path = self._images_path + "cheeky_face.jpg"

        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self._limb)
        self._baxter_gripper = baxter_interface.Gripper(self._limb)
        self._baxter_head = baxter_interface.Head()
        self._baxter_head.set_pan(0.0)
        print "Calibrating gripper..."
        self._baxter_gripper.calibrate()

        self._mill_pos = {}
        self._picking_pos = {}
        self.picked = False
        self._is_pickable = False
        self._nodded = False

        if (not self._check_config()):
            exit(0)
        self._read_config(self._config_file_path)

    def _get_mill_pos(self, x, y, limb):
        alph = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        acceptable = {'left': ['a1', 'a4', 'a7', 'b2', 'b4', 'b6', 'c3',
                               'c4', 'c5', 'd1', 'd2', 'd3', 'd5', 'd6',
                               'd7', 'e3', 'e4', 'e5', 'f2', 'f4', 'f6',
                               'g1', 'g4', 'g7']}
        mill_x = alph[x]
        mill = mill_x + str(y+1)
        if mill in acceptable[limb]:
            return mill
        else:
            return None

    def _check_config(self):
        ri = ""
        while (1):
            ri = raw_input("Have you calibrated the arm? [y/n] ")
            if ri.lower() == "y":
                print "Awesome! Carry on."
                return True
            elif ri.lower() == "n":
                print ">> run `rosrun baxter_mill calibrate.py` first! <<"
                return False

    def _read_config(self, file):
        """
        Read positions from config file.
        """
        print "Reading positions from file."
        f = open(file, 'r')
        lines = f.readlines()
        splitted = [line.split("=") for line in lines]
        for x in range(1,10):
            t = "p" + str(x)
            self._picking_pos[t] = eval(splitted.pop(0)[1])
        self._neutral_pos = eval(splitted.pop(0)[1])
        for x in range(7):
            for y in range(7):
                t = self._get_mill_pos(x, y, self._limb)
                if not t:
                    continue
                self._mill_pos[t] = eval(splitted.pop(0)[1])
        print "Positions are in memory."
        f.close()

    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        # rospy.sleep(1)

    def head_turn(self, direction=-1):
        """
        -1 = left, 1 = right
        """
        self._baxter_head.set_pan(direction*0.8, 50)
        self._baxter_head.set_pan(0.0, 10)

    def gripper_open(self, percentage):
        if percentage < 100:
            return self._baxter_gripper.command_position(percentage)
        else:
            return False

    def pick(self, pos):
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        if pos[0] == "p":
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][1])
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][0])
            self._baxter_gripper.close()
            rospy.sleep(0.2)
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][1])
        else:
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][1])
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][0])
            self._baxter_gripper.close()
            rospy.sleep(0.2)
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][1])

    def release(self, pos):
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        if pos[0] == "p":
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][1])
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][0])
            self.gripper_open(50)
            rospy.sleep(0.2)
            self._baxter_limb.move_to_joint_positions(self._picking_pos[(pos)][1])
        else:
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][1])
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][0])
            self.gripper_open(50)
            rospy.sleep(0.2)
            self._baxter_limb.move_to_joint_positions(self._mill_pos[(pos)][1])
