import logging
import argparse

import rospy

import baxter_interface
from sensor_msgs.msg import Image

import cv2
import cv_bridge

import math

logger = logging.getLogger('baxterRubiks')


class BaxterInterfaceTool(object):
    """
    Uses as many of the baxter interface tools as possible, will help with
    creating the manipulations
    """

    def __init__(self, loglevel):
        """
        initializes the object
        """
        # Initialize the rospy node
        rospy.init_node('baxter_interface_tool')
        # Register the clean shutdown function, which is called before
        # shutting down
        rospy.on_shutdown(self.clean_shutdown)

        # Change Logger level
        self.logger_change_level(loglevel)

        # Create baxter_interface limb instances
        self.limb_left = baxter_interface.Limb('left')
        self.limb_right = baxter_interface.Limb('right')
        self.gripper_left = baxter_interface.Gripper('left')
        self.gripper_right = baxter_interface.Gripper('right')

        # Create limb positions container
        self.left_positions = list()
        self.right_positions = list()
        # Create variable to keep track of recording state
        self.recording_positions = False

        # Verify robot is enabled
        logger.info('Getting robot state')
        self.robotstate = baxter_interface.RobotEnable()
        self.initial_state = self.robotstate.state().enabled
        logger.info('Enabling robot')
        self.robotstate.enable()

        # Calibrate the grippers
        self.gripper_left.calibrate()
        self.gripper_right.calibrate()

        # Create Navitaors I/O
        self.left_navigator_io = baxter_interface.Navigator('left')
        self.right_naviagator_io = baxter_interface.Navigator('right')

        # Image path to display on baxters face camera
        self.img_path = 'rubiks_algorithm_image.jpg'

    def logger_change_level(self, loggerlevel):
        """
        Changes the baxterRubiks loggers level based on user input
        """
        if loggerlevel:
            logger.setLevel('DEBUG')
            logger.handlers[0].setLevel('DEBUG')
        else:
            logger.setLevel('INFO')
            logger.handlers[0].setLevel('INFO')

    def record_positions(self):
        """
        function to record the positions of the arms
        """
        logger.info('record positions begun')
        # Connect the navigator io signals to the callback functions
        # on scroll wheel press
        self.left_navigator_io.button0_changed.connect(self.record_position_left)
        self.right_naviagator_io.button0_changed.connect(self.record_position_right)
        # on navigator rething button press (either arm)
        self.left_navigator_io.button1_changed.connect(self.stop_recording)
        self.right_naviagator_io.button1_changed.connect(self.stop_recording)

        # Set the recording flag
        self.recording_positions = True

        # Loop until waypoints are done being recorded
        while self.recording_positions and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # Release the navigator I/O signals, disconnecting the callback functions
        self.left_navigator_io.button0_changed.disconnect(self.record_position_left)
        self.right_naviagator_io.button0_changed.disconnect(self.record_position_right)
        self.left_navigator_io.button1_changed.disconnect(self.stop_recording)
        self.right_naviagator_io.button1_changed.disconnect(self.stop_recording)
        logger.info('record prositions finished')

    def playback_positions(self):
        """
        Plays back the recorded positions once
        """
        rospy.sleep(1.0)

        logger.info('Position playback begun')

        while not rospy.is_shutdown():
            for position in self.left_positions:
                if rospy.is_shutdown():
                    break
                self.limb_left.move_to_joint_positions(position, timeout=20.0)

        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            for position in self.right_positions:
                if rospy.is_shutdown():
                    break
                self.limb_right.move_to_joint_positions(position, timeout=20.0)
        logger.info('position playback finished')

    def gripper_motions(self):
        """
        Class to close grippers, rotate several ways, then open again
        Should cover all the gripper code requried for manipulations
        """
        logger.info('gripper motions begun')
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting program')
            return
        # ----- Using left gripper now -----
        logger.debug('moving left gripper')
        self.gripper_left.close()
        rospy.sleep(0.5)
        left_angles = self.limb_left.joint_angles()
        # 90 clockwise
        left_angles['left_w2'] = 1.57
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # 90 anticlockwise
        left_angles['left_w2'] = -1.57
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # 180 clockwise
        left_angles['left_w2'] = 3.14
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # return to original position
        left_angles['left_w2'] = 0.0
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # Open gripper
        self.gripper_left.open()
        rospy.sleep(3.0)
        # ----- Using right gripper now ----
        logger.debug('moving right gripper')
        self.gripper_right.close()
        rospy.sleep(0.5)
        right_angles = self.limb_right.joint_angles()
        # 90 clockwise
        right_angles['right_w2'] = 1.57
        self.limb_right.move_to_joint_positions(right_angles)
        rospy.sleep(0.5)
        # 90 anticlockwise
        right_angles['right_w2'] = -1.57
        self.limb_right.move_to_joint_positions(right_angles)
        rospy.sleep(0.5)
        # 180 clockwise
        right_angles['right_w2'] = 3.14
        self.limb_right.move_to_joint_positions(right_angles)
        rospy.sleep(0.5)
        # return to original position
        right_angles['right_w2'] = 0.0
        self.limb_right.move_to_joint_positions(right_angles)
        rospy.sleep(0.5)
        # Open gripper
        self.gripper_right.open()
        rospy.sleep(3.0)
        logger.info('gripper motions finished')

    def pickup_cube(self):
        """
        Picks up the cube from the preset position
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting')
            return
        # Define the angles for the 3 positions needed
         # angles for the position to pick up the cube
        cube_pickup = {
            'right_e0': -0.7669903930664063,
            'right_e1': 0.8007379703613282,
            'right_s0': 1.2172137537963867,
            'right_s1': -0.4421699616027832,
            'right_w0': 0.6803204786499024,
            'right_w1': 1.432738054248047,
            'right_w2': -0.120800986907959}

        # angles for position above the cube
        above_cube_pickup = {
            'right_e0': -0.33402431618041994,
            'right_e1': 0.832568071673584,
            'right_s0': 1.0538448000732423,
            'right_s1': -0.6876068873840332,
            'right_w0': 0.26461168560791015,
            'right_w1': 1.4638011651672365,
            'right_w2': -0.016490293450927736}

        # angles for a central position
        right_centered = {
            'right_e0': -0.11926700612182618,
            'right_e1': 2.259937193170166,
            'right_s0': 0.607839886505127,
            'right_s1': -1.145883647241211,
            'right_w0': 1.36485940446167,
            'right_w1': 1.746053629815674,
            'right_w2': -1.1727283109985351}

        left_centred = {
            'left_e0': -0.33785926814575196,
            'left_e1': 2.086213869140625,
            'left_s0': -0.10584467424316407,
            'left_s1': -1.0837574254028322,
            'left_w0': -1.125174906628418,
            'left_w1': 1.6751070184570314,
            'left_w2': -0.645422415765381}

        left_face_rotation = {
            'left_e0': -0.33172334500122075,
            'left_e1': 2.148340090979004,
            'left_s0': -0.2546408104980469,
            'left_s1': -1.1201894690734864,
            'left_w0': -1.2275681241027834,
            'left_w1': 1.601092445526123,
            'left_w2': -0.645422415765381}

        logger.info('picking up the rubiks cube')
        rospy.sleep(5)
        self.limb_right.move_to_joint_positions(right_centered)
        rospy.sleep(0.5)
        logger.info('moving to above pickup')
        self.limb_right.move_to_joint_positions(above_cube_pickup)
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(cube_pickup)
        rospy.sleep(0.5)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(above_cube_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(right_centered)
        rospy.sleep(0.5)

        self.limb_left.move_to_joint_positions(left_centred)
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(left_face_rotation)
        rospy.sleep(0.5)

        logger.debug('moving left gripper')
        self.gripper_left.close()
        rospy.sleep(0.5)
        left_angles = self.limb_left.joint_angles()
        # 90 clockwise
        left_angles['left_w2'] += math.pi / 2
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # 90 anticlockwise
        left_angles['left_w2'] -= math.pi / 2
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # 180 clockwise
        left_angles['left_w2'] += math.pi
        self.limb_left.move_to_joint_positions(left_angles)
        rospy.sleep(0.5)
        # Open gripper
        self.gripper_left.open()
        rospy.sleep(0.5)
        left_centred['left_w2'] = self.limb_left.joint_angle('left_w2')
        self.limb_left.move_to_joint_positions(left_centred)
        rospy.sleep(0.5)
        left_centred['left_w2'] = -0.645422415765381
        self.limb_left.move_to_joint_positions(left_centred)
        rospy.sleep(0.5)

        logger.info('putting cube back down')
        self.limb_right.move_to_joint_positions(above_cube_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(cube_pickup)
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(above_cube_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(right_centered)
        rospy.sleep(0.5)
        logger.info('pickup and place done!')

    def send_image(self):
        """
        Send the image specified to the baxter head screen
        """
        logger.debug('send image begun')
        img = cv2.imread(self.img_path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        #Sleep to allow the image to be published
        rospy.sleep(1)
        logger.debug('send image finished')

    def record_position_left(self, value):
        """
        Stores the current left arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for left arm recorded')
            self.left_positions.append(self.limb_left.joint_angles())

    def record_position_right(self, value):
        """
        Stores the current right arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for right arm recorded')
            self.right_positions.append(self.limb_right.joint_angles())

    def stop_recording(self, value):
        """
        Sets recording_positions to false and ends the recording loop
        """
        if value:
            self.recording_positions = False

    def clean_shutdown(self):
        logger.info('Exiting')
        if not self.initial_state:
            logger.info('Disabling Robot')
            self.robotstate.disable()
        return True


def parse_arguments():
    """
    Sets up argument parsing
    """
    parser = argparse.ArgumentParser(prog='baxter-interface-tool',
                                     description='tool using the baxter interface api',
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-v', '--verbose', action='store_true', help='increase verbosity')
    return parser.parse_args()


def main():
    """
    Gets command line arguments, creates a BaxterInterfaceTool object and run control function
    """
    args = parse_arguments()
    baxter_interface_tool = BaxterInterfaceTool(args.verbose)
    logger.info('baxter interface tools begun')
    # baxter_interface_tool.send_image()
    # baxter_interface_tool.record_positions()
    # baxter_interface_tool.playback_positions()
    # baxter_interface_tool.gripper_motions()
    baxter_interface_tool.pickup_cube()
    logger.info('baxter interface tools finished')


if __name__ == '__main__':
    main()
