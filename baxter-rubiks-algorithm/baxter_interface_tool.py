import logging
import argparse

import rospy

import baxter_interface

import cv2
import cv_bridge


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

        # Create baxter_interface limb instances
        left_limb = baxter_interface.Limb('left')
        right_limb = baxter_interface.Limb('right')
        grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

        # Create limb positions container
        self.left_positions = list()
        self.right_positions = list()

        # Verify robot is enabled
        logger.info('Getting robot state')
        self._robotstate = baxter_interface.RobotEnable()
        self._initial_state = self._robotstate.state().enabled
        logger.info('Enabling robot')
        self._robotstate.enable()

        # Create Navitaors I/O
        self._left_navigator_io = baxter_interface.Navigator('left')
        self._right_naviagator_io = baxter_interface.Navigator('right')

        # Image path to display on baxters face camera
        self._img_path = 'rubiks_algorithm_image.jpg'

    def setup_logger(self, loggerlevel):
        """
        Sets up the logging with the specified loggerlevel
        """
        logformat = '%(asctime)s:%(message)s'
        if loggerlevel:
            logging.basicConfig(level=logging.DEBUG, format=logformat)
        else:
            logging.basicConfig(level=logging.INFO, format=logformat)

    def send_image():
        """
        Send the image specified to the baxter head screen
        """
        img = cv2.imread(self._img_path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_side=1)
        pub.publish(msg)
        #Sleep to allow the image to be published
        rospy.sleep(1)

    def _record_position_left(self, value):
        """
        Stores the current left arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for left arm recorded')
            self.left_positions.append(self.left_limb.joint_angles())

    def _record_position_right(self, value):
        """
        Stores the current right arm position when the
        Navigator OK button is pressed
        """
        if value:
            logger.info('angles for right arm recorded')
            self.right_positions.append(self.right_limb.joint_angles())


def parse_arguments():
    """
    Sets up argument parsing
    """
    parser = argparse.ArgumentParser(prog='baxter-interface-tool',
                                     description='tool using the baxter interface api',
                                     formatter_class=argeparse.RawTextHelpFormatter)
    parser.add_argument('-v', '--verbose', action='store_true', help='increase verbosity')
    return parser.parse_args()


def main():
    """
    Gets command line arguments, creates a BaxterInterfaceTool object and run control function
    """
    args = parse_arguments()
    baxter_interface_tool = BaxterInterfaceTool(args.verbose, args.limb)
    baxter_interface_tool."here needs the control function"()

if __name__ == '__main__':
    main()
