# -*- coding: utf-8 -*-
import subprocess
import commands
import time
import cv2
# import numpy as np
import requests
import argparse
# from matplotlib import pyplot as plt
import logging
from collections import Counter

import rospy
import baxter_interface
from sensor_msgs.msg import Image
import cv_bridge

logger = logging.getLogger('baxterRubiks')
logging.getLogger('requests').setLevel(logging.WARNING)


# classes : solve cube, cube explorer, vision system, manipulations, a class to act as variable storage for each face
# (create an instance for each face required)
# --------------------------------------------------------------------------------
# BaxterRubiks: main control class, contain main control code, logging,
# convert_to_singmaster
# --------------------------------------------------------------------------------
# CubeExplorer: contains check_solver_status, run_solver, exit_solver,
# check_solver_connection, send_solver_face_encoding,
# --------------------------------------------------------------------------------
# VisionAnalysis: get images from camera, general vision system manipulations
# --------------------------------------------------------------------------------
# Baxter: controls communication and commands for baxter
# --------------------------------------------------------------------------------
# CubeFace: contains variables for each face cubie value (on instance per face)
# --------------------------------------------------------------------------------


class BaxterRubiks(object):
    """
    Solves a rubiks cube using the Baxter Research Robot's vision and servo system
    along with the solving algorithm Cube Explorer
    This class specifically controls the overall process and creates instances of
    the other classes.
    """

    def __init__(self, loglevel):
        """
        initializes object
        """
        self.change_logger_level(loglevel)

    def change_logger_level(self, loggerlevel):
        """
        Sets up the logging with the specified level
        """
        if loggerlevel:
            logger.setLevel('DEBUG')
            logger.handlers[0].setLevel('DEBUG')
        else:
            logger.setLevel('INFO')
            logger.handlers[0].setLevel('INFO')

    def solve_rubiks_cube(self):
        """
        Function that automates the process of solving the rubiks cube using
        Baxter
        """
        logger.info('-----START-----')

        # create required instances of each class
        cube_solver = CubeExplorer()
        # not vision system ATM
        # vision_system = VisionAnalysis()
        baxter = Baxter()
        front_face = CubeFace()
        back_face = CubeFace()
        up_face = CubeFace()
        down_face = CubeFace()
        left_face = CubeFace()
        right_face = CubeFace()
        logger.debug('instances created')
        # send the program image to baxters face
        baxter.display_image()
        # run cube solver
        cube_solver.run_solver()
        # check the connection to Cube Explorer
        if cube_solver.check_solver_connection() is False:
            return False
        # pick up the rubiks cube
            # placeholder for the manouver ------------------------------
        # move cube to infront of baxter camera
            # placeholder for manouver ------------------------------
        # scan each face, storing the image in the corresponding CubeFace class
            # get the face orientated towards the camera
            # scan and save the image into each face instance
        # send face images to the vision system for analysis
            # send the faces self.face_image to the vision system, save the results into self.cublet_colours
        # test the colours from the vision system
        cube_colours = [front_face.cubelet_colours, back_face.cubelet_colours, left_face.cubelet_colours,
                        right_face.cubelet_colours, up_face.cubelet_colours, down_face.cubelet_colours]
        if self.colour_test(cube_colours):
            logger.debug('cube colours test successful')
        else:
            logger.error('cube colours are invalid')
            return False
        # convert the cube colours into singmaster notation
        cube_colours_singmaster = self.convert_to_singmaster(cube_colours)
        logger.debug('cube colours converted to singmaster notation')
        # send face encoding to Cube Explorer
        cube_solver.send_solver_face_encoding(cube_colours_singmaster)
        # perform each manipulation
            # placeholder for manipulations -----------------------------
        # place down rubiks cube
            # placeholder for manipulations -----------------------------
        # cleanup - exit cube explorer, disable baxter ect...
        logger.info('-----END-----')

    def convert_to_singmaster(self, cube_colours):  # needs changing when the cube_colours is ordered properly
        """
        Converts face colour encoding to singmaster notation
        Outputs as a string in the order required for Cube Explorer:
        up - right - front - down - left - back
        """
        # reorders cube_colours ----------------------- maybe save in propper order first time around!?!?!?
        # this asumes the ordering is front - back - left - right - up - down
        # just remove the following statement when cube_colours is ordered properly.....
        required_order = [2, 5, 0, 3, 4, 1]
        cube_colours = [cube_colours[i] for i in required_order]
        # get the centre cubelet colour of each face
        front_colour = cube_colours[0][4]
        back_colour = cube_colours[1][4]
        up_colour = cube_colours[2][4]
        down_colour = cube_colours[3][4]
        left_colour = cube_colours[4][4]
        right_colour = cube_colours[5][4]
        # combines face colour strings into one string
        cube_colours_joined = ''.join(cube_colours)
        # replaces colour letter with its corresponding face letter
        cube_colours_singmaster = cube_colours_joined.replace(front_colour, 'f').replace(back_colour, 'b').replace(up_colour, 'u').replace(down_colour, 'd').replace(left_colour, 'l').replace(right_colour, 'r')
        return cube_colours_singmaster

    def colour_test(self, cube_colours):
        """
        Logical tests to check if colour values from the vision system are valid
        cube_colours is a list of the face colours
        """
        # test if each cube face has 9 colours
        if any(len(x) is not 9 for x in cube_colours):
            logger.error('not nine colours in each face')
            return False
        # test if there is nine of each colour
        elif any(x is not 9 for x in Counter(cube_colours).values()):
            logger.error('not nine of each colour')
            return False
        # test if each centre is unique
        elif any(x is not 1 for x in Counter(cube_colours[0, 1, 2, 3, 4, 5][4]).values()):
            logger.error('centre colours are not unique')
            return False
        # all tests have passed
        else:
            return True


class CubeExplorer(object):
    """
    Manages running and communication between the algorithm and Cube Explorer
    """

    def __init__(self):
        # initialize the class variable solver, which will later be set to a
        # popen object.
        self.solver = None

    def check_solver_status(self):
        """
        checks whether the solver is running
        """
        processes = commands.getoutput('ps-A')
        if 'cube512htm' in processes:
            logger.debug('cube explorer running')
            return True
        else:
            logger.debug('cube explorer not running')
            return False

    def run_solver(self):
        """
        runs Cube Explorer if it is not already running
        """
        if self.check_solver_status() is False:
            self.solver = subprocess.Popen('solver/cube512htm.exe')
            logger.info('Cube Explorer started')
            return
        else:
            logger.info('Cube Explorer running')
            return True

    def exit_solver(self):
        """
        exits Cube Explorer
        """
        if self.solver is None:
            logger.info('Cube Explorer was initially open, leaving open')
        else:
            close_cube_explorer = raw_input('Close Cube Explorer? (Y/N)')
            if close_cube_explorer == 'y':
                logger.info('Now exiting')
                subprocess.Popen.kill(self.solver)
                subprocess.Popen.wait(self.solver)
            elif close_cube_explorer == 'n':
                logger.info('Leaving open')
            else:
                logger.info('Invalid input, now exiting')

    def check_solver_connection(self):
        """
        Checks the connection to Cube Explorers webserver
        """
        logger.info('Checking connection to Cube Explorer')
        # Define loop variable to allow a timeout
        loop = 12
        while loop > 0:
            try:
                # send status request to the webserver
                requests.get('http://127.0.0.1:8081/?status')
                logger.info('Connection successful')
                return True
            except requests.exceptions.ConnectionError:
                time.sleep(2.5)
                loop += 1
        else:
            logger.error('Couldn\'t connect to webserver')
            return False

    def send_solver_face_encoding(self, face_encoding):
        """
        Sends the face encoding to the solver, must already be in singmaster
        notation. Returns the manouvers as a list without html encoding.
        """
        try:
            response = requests.get('http://127.0.0.1:8081/?' + face_encoding)
            # clear the main window
            requests.get('http://127.0.0.1:8081/?clear')
        except requests.exceptions.ConnectionError:
            return False
        manouvers_raw = response.text
        # removes the html encoding from the response
        manouvers = manouvers_raw.replace('<HTML><BODY>\r\n', '')
        manouvers = manouvers.replace('\r\n</BODY></HTML>\r\n', '')
        # check to see if the cube was solved
        if 'Cube cannot be solved' in manouvers:
            logger.error('{}'.format(manouvers))
            return False
        else:
            # returns manouvers as a list
            return manouvers.split()


'''
class VisionAnalysis(object):
    """
    Analyses images of cube faces and returns colour values
    Will handle getting the image from the camera feed too.
    """

    def __init__(self):
        pass

    def get_camera_image(self):
        """
        Gets the image from the connected camera, returns image as a mat in RGB
        NOTE: Baxter camera needs to be mounted as a webcamera, function to do this
        will be included in this class
        """
        cap = cv2.VideoCapture(0)
        i = 0
        while i < 10:
            ret, frame = cap.read()
            time.sleep(0.001)
            i += 1
        # changes from BGR to RGB
        facemat = frame[:, :, ::-1]
        # when finished, frame capture is released
        cap.release()
        return facemat

    def vision_system(self):
        """
        Analises the face images, returns equivalent colour values
        """
        # this function is still a placeholder !!!!!!!
        ## Currently just parses preset values and as such is a placeholder
        ## for the actual vision system
        ## will probably need to be broken into multiple functions
        # variables obtained by the vision system
        # colours are:
        #            o = orange
        #            n = blue
        #            r = red
        #            g = green
        #            w = white
        #            y = yellow
        # NOTE # Do not match colour letter and face letter, causes .replace issues
        frontvs = 'rnwgrwoon'  # no'
        backvs = 'nwgyoowyr'
        upvs = 'roygynyyn'
        downvs = 'grwwwgwrg'
        leftvs = 'ywngnygny'
        rightvs = 'roorgonro'

        ## will be used to get images
        ## get all of the faces, each one needs to call get_camera_image
        #print('Show FRONT:'),
        front = self.get_camera_image()
        #print('Done')
        #print('Show BACK:'),
        back = self.get_camera_image()
        #print('Done')
        #print('Show UP:'),
        up = self.get_camera_image()
        #print('Done')
        #print('Show DOWN:'),
        down = self.get_camera_image()
        #print('Done')
        #print('Show LEFT:'),
        left = self.get_camera_image()
        #print('Done')
        #print('Show RIGHT:'),
        right = self.get_camera_image()
        #print('Done')
        # setup a matplot with all the faces (arranged like the cube is folded out)
        titles = ['Front', 'Back', 'Up', 'Down', 'Left', 'Right']
        images = [front, back, up, down, left, right]
        position = [5, 3, 2, 8, 4, 6]
        for i in range(6):
            plt.subplot(3, 3, position[i]), plt.imshow(images[i])
            plt.title(titles[i])
            plt.xticks([]), plt.yticks([])
        plt.show()

        return frontvs, backvs, upvs, downvs, leftvs, rightvs
'''


class Baxter(object):
    """
    Controls Baxters arms while manipulating the cube.
    Functions will include: get the cube, place back the cube,
                            scan the cubes faces, roate the cube faces
                            rotate the cube itself, move cube to a central area
    """
    def __init__(self):
        """
        Initializes rospy nodes and enables the robot
        """
        # Initialize the rospy node
        rospy.init_node('baxter_rubiks')
        # Register clean shutdown function, called before rospy shuts down
        rospy.on_shutdown(self.clean_shutdown)

        # Create limb instances
        self.limb_left = baxter_interface.Limb('left')
        self.limb_right = baxter_interface.Limb('right')
        self.gripper_left = baxter_interface.Gripper('left')
        self.gripper_right = baxter_interface.Gripper('right')

        # Verify the robot is enabled
        logger.info('Getting robot state')
        self.robotstate = baxter_interface.RobotEnable()
        self.initial_state = self.robotstate.state().enabled
        logger.info('Enabling robot')
        self.robotstate.enable()

        # Calibrate the grippers
        self.gripper_left.calibrate()
        self.gripper_right.calibrate()

        # Image path to display on baxters face screen
        self.img_path = 'rubiks_algorithm_image.jpg'

    def clean_shutdown(self):
        """
        Function for safely shutting the program down
        """
        logger.info('Exiting clean')
        if not self.initial_state:
            logger.info('Disabling Robot')
            self.robotstate.disable()
        return True

    def display_image(self):
        """
        Displys the image specified to baxters face/screen
        """
        logger.debug('sending image')
        img = cv2.imread(self.img_path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow the image to publish
        rospy.sleep(1)
        logger.debug('send image completed')

    def rotate_gripper(self, manouver):
        """
        Rotates the left gripper the ammount specified
        manouver: the angle to rotate the gripper:
        {face} = 90 degrees clockwise
        {face}' = 90 degrees anticlockwise
        {face}2 = 180 degrees clockwise
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting')
            return
        faceQuaterCW = ['F', 'B', 'L', 'R', 'U', 'D']
        faceQuaterACW = ["F'", "B'", "L'", "R'", "U'", "D'"]
        faceHalfCW = ['F2', 'B2', 'L2', 'R2', 'U2', 'D2']
        joint_angles = self.limb_left.joint_angles()
        if manouver in faceQuaterCW:
            joint_angles['left_w2'] = 1.57
        elif manouver in faceQuaterACW:
            joint_angles['left_w2'] = -1.57
        elif manouver in faceHalfCW:
            joint_angles['left_w2'] = 3.14
        else:
            logger.error('invalid gripper manouver')
            return False
        logger.debug('moving to {}'.format(joint_angles))
        self.limb_left.move_to_joint_positions(joint_angles)

    def rotate_cube(self, completed_move, next_move):
        """
        Rotates the cube so the required face is avaliable
        completed_move = previous move done
        next_move = current move to be performed
        """
        # Will need to define 30? manouvers

    def pickup_cube(self):
        """
        Picks up the cube from the preset position
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting')
            return
        # Define the angles for the 3 positions needed
        angles_pickup = {'right_e0': -0.4506068559265137,
                         'right_e1': 0.8179952542053224,
                         'right_s0': 1.034286545050049,
                         'right_s1': -0.5537670637939454,
                         'right_w0': 0.3501311144348145,
                         'right_w1': 1.3583399861206056,
                         'right_w2': 1.4699370883117677}

        angles_above_pickup = {'right_e0': -0.6043884297363281,
                               'right_e1': 0.9046651686218262,
                               'right_s0': 1.138213743310547,
                               'right_s1': -0.7094661135864259,
                               'right_w0': 0.37083985504760747,
                               'right_w1': 1.4150972752075197,
                               'right_w2': 1.4776069922424317}

        angles_centered = {'right_e0': -0.2803349886657715,
                           'right_e1': 2.0942672682678225,
                           'right_s0': 0.6810874690429688,
                           'right_s1': -0.39231558605346684,
                           'right_w0': 1.3602574621032715,
                           'right_w1': 1.8058788804748536,
                           'right_w2': -1.7648448944458008}
        logger.info('picking up the rubiks cube')
        self.limb_right.move_to_joint_positions(angles_above_pickup)
        self.gripper_right.open()
        self.limb_right.move_to_joint_positions(angles_pickup)
        self.gripper_right.close()
        self.limb_right.move_to_joint_positions(angles_above_pickup)
        self.limb_right.move_to_joint_positions(angles_centered)


class CubeFace(object):
    """
    A class for variable storage along with relevant functions:
        get the middle colour, send back a string containing all the cubelet values
    """
    def __init__(self):
        self.cubelet_colours = list
        self.face_image = list

    def GetCentreCubletColour(self):
        return self.cubelet_colours[4]

    def ReturnFaceColours(self):
        return self.cubelet_colours


def parse_arguments():
    """
    sets up argument parsing
    """
    parser = argparse.ArgumentParser(prog='baxter-rubiks-algorithm',
                                     description=('Solves a rubiks cube using the baxter research ' +
                                                  'robot and a rubiks solver'),
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-v', '--verbose', action='store_true', help='increases verbosity')
    return parser.parse_args()


def main():
    """
    Gets command line arguments, creates a BaxterRubiks object and runs the control function
    """
    args = parse_arguments()
    cube_solve = BaxterRubiks(args.verbose)
    cube_solve.solve_rubiks_cube()

if __name__ == '__main__':
    main()
