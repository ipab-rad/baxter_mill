#!/usr/bin/python

import sys
import rospy
import rospkg
import baxter_interface
# import tf
from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    # Quaternion,
)

from std_msgs.msg import Header

# from sensor_msgs.msg import (
#     # Image,
#     # JointState,
#     )

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class Calibrate(object):
    """
    This class defines the calibration for the arm (which by default
    is the left bottom_pos).
    """

    def __init__(self, limb='left'):
        self._rp = rospkg.RosPack()
        self._config_path = self._rp.get_path('baxter_mill') + '/config/'

        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self._limb)
        self._neutral_pos = {}
        self._picking_pos = {}
        self._square_length = 50  # mm - this will also be the offset

        self._neutral_bool = False
        self._default_bool = True

        self._mill_pos = {}  # dict of list (tuple) of dict (sigh)

        self.pick_pos = {}  # picking position
        self.br_pos = {}  # bottom right position (7,0)
        self._the_pose = Pose()
        self._should_io = baxter_interface.DigitalIO(self._limb +
                                                     '_shoulder_button')
        self._dash_io = baxter_interface.DigitalIO(self._limb +
                                                   '_upper_button')
        self._circle_io = baxter_interface.DigitalIO(self._limb +
                                                     '_lower_button')

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        self._circle_io.state_changed.connect(self._default_points)
        self.done_calibration = False

    def _find_joint_position(self, pose, x_off=0.0, y_off=0.0, z_off=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it (to avoid opening the structure all the time
        outside of the function).
        '''

        ik_request = SolvePositionIKRequest()
        the_pose = deepcopy(pose)
        the_pose['position'] = Point(x=pose['position'].x + x_off,
                                     y=pose['position'].y + y_off,
                                     z=pose['position'].z + z_off)
        approach_pose = Pose()
        approach_pose.position = the_pose['position']
        approach_pose.orientation = the_pose['orientation']
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ik_request.pose_stamp.append(pose_req)
        resp = self._iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _blink_light(self, io_component="right_itb_light_outer"):
        """
        Blinks a Digital Output on then off.
        """

        b = baxter_interface.digital_io.DigitalIO(io_component)
        print "Initial state: ", b.state
        # One second should be enough to notice it
        b.set_output(True)
        rospy.sleep(1)
        b.set_output(False)
        rospy.sleep(1)

    def _default_points(self, value):
        """
        Registers the picking point
        """
        print "########## Clicked!"
        if value:
            if len(self._picking_pos) == 0 and self._limb == "left":
                self._blink_light()
                # Record default position
                print 'Recording picking location'
                self._picking_pos[0] = self._baxter_limb.joint_angles()
                self._picking_pos[1] = self._find_joint_position(
                    self._baxter_limb.endpoint_pose(),
                    z_off=0.10)
            # Registers the central neutral point. Otherwise the arm
            # could move somewhere below the table while moving to the
            # chessboard positions.
            elif len(self._neutral_pos) == 0:
                self._blink_light()
                # Record neutral position
                print 'Recording neutral location'
                self._neutral_pos = self._baxter_limb.joint_angles()
                self._neutral_bool = False
            # Registers the (0,0) point *just* above the table
            # (1|2mm). Will be used to generate all the rest of
            # points.
            elif len(self.br_pos) == 0:
                self._blink_light()
                print 'Recording pick location'
                self.br_pos[0] = self._baxter_limb.joint_angles()
                self._the_pose = self._baxter_limb.endpoint_pose()
                self.br_pos[1] = self._find_joint_position(
                    self._the_pose,
                    z_off=0.10)
            else:
                print "Stop pressing! You have already calibrated!"

    def get_mill_pos(self, x, y, limb):
        alph = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        acceptable = {'right': ['a1', 'a4', 'a7', 'b2', 'b4', 'b6', 'c3',
                                'c4', 'c5', 'd1', 'd2', 'd3', 'd5', 'd6',
                                'd7'],
                      'left': ['e3', 'e4', 'e5', 'f2', 'f4', 'f6', 'g1',
                               'g4', 'g7']}
        mill_x = alph[x]
        mill = mill_x + str(y+1)
        if mill in acceptable[limb]:
            return mill
        else:
            return None

    def generate_positions(self):
        """
        Generates positions given position 0,0 has been registered.
        WARNING: Make sure chessboard is parallel to robot.
        Returns a list of non-generated positions (most likely to be empty)
        """

        if len(self.br_pos) != 0:
            missed_pos = []
            cur_bottom_pose = self._the_pose
            for y in range(8):
                for x in range(8):
                    x_o = 0.065*x
                    y_o = y * 0.065 * -1
                    t = (7 - y, 7 - x)  # yep
                    bottom_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o
                    )
                    top_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o,
                        z_off=0.10
                    )
                    rospy.sleep(0.1)
                    self._mill_pos[t] = [bottom_pos, top_pos]
                    if len(self._mill_pos[t][0]) == 0:
                        missed_pos.append((t, "bottom"))
                    if len(self._mill_pos[t][1]) == 0:
                        missed_pos.append((t, "top"))
            return missed_pos

    def generate_right_positions(self):
        """
        Generates positions given position a,1 has been registered.
        WARNING: Make sure chessboard is parallel to robot.
        Returns a list of non-generated positions (most likely
        to be empty)
        """

        if len(self.br_pos) != 0:
            missed_pos = []
            cur_bottom_pose = self._the_pose
            for y in range(6, -1, -1):
                for x in range(4):
                    x_o = (6 - y) * 0.065
                    y_o = 0.065*x
                    # t = (7 - y, 7 - x)  # yep
                    t = self.get_mill_pos(x, y, self._limb)
                    if not t:
                        continue
                    print t
                    bottom_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o
                    )
                    top_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o,
                        z_off=0.10
                    )
                    rospy.sleep(0.1)
                    self._mill_pos[t] = [bottom_pos, top_pos]
                    if len(self._mill_pos[t][0]) == 0:
                        missed_pos.append((t, "bottom"))
                    if len(self._mill_pos[t][1]) == 0:
                        missed_pos.append((t, "top"))
            return missed_pos

    def generate_left_positions(self):
        """
        Generates positions given position e,1 has been registered.
        WARNING: Make sure chessboard is parallel to robot.
        Returns a list of non-generated positions (most likely
        to be empty)
        """

        if len(self.br_pos) != 0:
            missed_pos = []
            cur_bottom_pose = self._the_pose
            for y in range(7):
                for x in range(4,7):
                    x_o = 0.05*x
                    y_o = y * 0.05 * -1
                    t = get_mill_pos(6-y, 6-x, self._limb)
                    bottom_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o
                    )
                    top_pos = self._find_joint_position(
                        cur_bottom_pose,
                        x_off=x_o,
                        y_off=y_o,
                        z_off=0.10
                    )
                    rospy.sleep(0.1)
                    self._mill_pos[t] = [bottom_pos, top_pos]
                    if len(self._mill_pos[t][0]) == 0:
                        missed_pos.append((t, "bottom"))
                    if len(self._mill_pos[t][1]) == 0:
                        missed_pos.append((t, "top"))
            return missed_pos

    def _save_config(self, file):
        """
        Saves positions to config file.
        """

        print "Saving your positions to file!"
        f = open(file, 'w')
        f.write('picking_pos=' + str(self._picking_pos) + '\n')
        f.write('neutral_pos=' + str(self._neutral_pos) + '\n')
        for x in range(7):
            for y in range(7):
                t = self.get_mill_pos(x, y, self._limb)
                if not t:
                    continue
                f.write(str(t) + "=" + str(self._mill_pos[t]) + '\n')
        f.close()

    def get_locations(self):
        """
        Main function of the class. Runs the calibrate procedure.
        """

        self.done_calibration = False
        while not self.done_calibration:
            self.read_file = raw_input("Are you sure you really want to"
                                       " overwrite your previous changes"
                                       "(y/n)? ")
            if self.read_file != 'y' and self.read_file != 'n':
                print "You must answer 'y' or 'n'"

            elif self.read_file == 'n':
                print "Alright then: using previous values."
                return
            else:
                if self._limb == "right":
                    self.calibrate_right()
                else:
                    self.calibrate_left()

    def calibrate_right(self):
        print ("Move the " + self._limb + " arm to the neutral "
               "position and press the circle button ")
        while(len(self._neutral_pos) == 0 and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        print ("Neutral gripping position - Registered.")

        print ("Move same arm to (a, 1) position and press the"
               "cirle button to record")
        while(len(self.br_pos) == 0 and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        print "Well done!"

        print "Starting generating positions"
        missed = self.generate_right_positions()
        print "Done generating positions"
        if len(missed) != 0:
            print "The IK generator has missed the following positions"
            print missed
            print "You will now repeat the calibration. Try again :)"
            # todo add manual partial calibration
        else:
            print "Saving your new configuration!"
            self._save_config(self._config_path + "right_positions.config")
            self.done_calibration = True

    def calibrate_left(self):
        print ("Move the " + self._limb + " arm to the default "
               "position (for picking) and "
               "press the circle button ")
        while(len(self._picking_pos) == 0 and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        print ("Default gripping position - Registered.")

        print ("Move the " + self._limb + " arm to the neutral "
               "position (for picking) and "
               "press the circle button ")
        while(len(self._neutral_pos) == 0 and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        print ("Neutral gripping position - Registered.")

        print ("Move same arm to (e,1) position and press the"
               "cirle button to record")
        while(len(self.br_pos) == 0 and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        print "Well done!"

        print "Starting generating positions"
        missed = self.generate_left_positions()
        print "Done generating positions"
        if len(missed) != 0:
            print "The IK generator has missed the following positions"
            print missed
            print "You will now repeat the calibration. Try again :)"
            # todo add manual partial calibration
        else:
            print "Saving your new configuration!"
            self._save_config(self._config_path + "left_positions.config")
            self.done_calibration = True


    def test(self):
        """
        Tests the four corners.
        """
        if not self.done_calibration:
            print "Calibrate the positions first!"
            return -1
        print "TESTING!"
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        print "Going to position a,1"
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('a1')][1])
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('a1')][0])
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        print "Going to position a,7"
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('a7')][1])
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('a7')][0])
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        print "Going to position d,1"
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('d1')][1])
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('d1')][0])
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)
        print "Going to position d,7"
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('d7')][1])
        self._baxter_limb.move_to_joint_positions(self._mill_pos[('d7')][0])
        self._baxter_limb.move_to_joint_positions(self._neutral_pos)


def main():
    rospy.init_node("baxter_mill_calibrate")
    rs = baxter_interface.RobotEnable()
    rs.enable()
    right = Calibrate("right")
    right.get_locations()
    right.test()
    left = Calibrate("left")
    left.get_locations()
    left.test()

if __name__ == "__main__":
    sys.exit(main())
