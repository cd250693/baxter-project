import logging
import argparse

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image

import cv2
import cv_bridge

logger = logging


class BaxterInterfaceTool(object):
    """
    Uses as many of the baxter interface tools as possible, will help with
    creating the manipulations
    """

    def __init__(self, loglevel):
        """
        initializes the object
        """
        self.loglevel = loglevel

        # Initialize the rospy node
        logger.info('initializing node')
        rospy.init_node('baxter_interface_tool')
        # Register the clean shutdown function, which is called before
        # shutting down
        rospy.on_shutdown(self.clean_shutdown)

        # Create baxter_interface limb instances
        self.left_limb = baxter_interface.Limb('left')
        self.right_limb = baxter_interface.Limb('right')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

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

        # Create Navitaors I/O
        self.left_navigator_io = baxter_interface.Navigator('left')
        self.right_naviagator_io = baxter_interface.Navigator('right')

        # Image path to display on baxters face camera
        self.img_path = 'rubiks_algorithm_image.jpg'

    def record_positions(self):
        """
        function to record the positions of the arms
        """
        logger.info('recording positions')
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
                self.left_limb.move_to_joint_positions(position, timeout=20.0)

        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            for position in self.right_positions:
                if rospy.is_shutdown():
                    break
                self.right_limb.move_to_joint_positions(position, timeout=20.0)

    def setup_logger(self, loggerlevel):
        """
        Sets up the logging with the specified loggerlevel
        """
        logformat = '%(asctime)s:%(message)s'
        if loggerlevel:
            logging.basicConfig(level=logging.DEBUG, format=logformat)
        else:
            logging.basicConfig(level=logging.INFO, format=logformat)

    def send_image(self):
        """
        Send the image specified to the baxter head screen
        """
        img = cv2.imread(self.img_path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_side=1)
        pub.publish(msg)
        #Sleep to allow the image to be published
        rospy.sleep(1)

    def record_position_left(self, value):
        """
        Stores the current left arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for left arm recorded')
            self.left_positions.append(self.left_limb.joint_angles())

    def record_position_right(self, value):
        """
        Stores the current right arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for right arm recorded')
            self.right_positions.append(self.right_limb.joint_angles())

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
    baxter_interface_tool.send_image()
    baxter_interface_tool.record_positions()
    baxter_interface_tool.playback_positions()


if __name__ == '__main__':
    main()
