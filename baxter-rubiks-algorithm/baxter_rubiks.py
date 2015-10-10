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
import math

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
        cube_colours_singmaster = cube_colours_joined.replace(
            front_colour, 'f').replace(
            back_colour, 'b').replace(
            up_colour, 'u').replace(
            down_colour, 'd').replace(
            left_colour, 'l').replace(
            right_colour, 'r')
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

        # Move both arms into the centre position
        self.limb_left.move_to_joint_positions(left_centred)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(right_centred)
        rospy.sleep(0.5)

        # Calibrate the grippers
        self.gripper_left.calibrate()
        self.gripper_right.calibrate()

        # Image path to display on baxters face screen
        self.img_path = 'rubiks_algorithm_image.jpg'

        # joint angles for the position to pick up the cube
        self.cube_pickup = {
            'right_e0': -0.7669903930664063,
            'right_e1': 0.8007379703613282,
            'right_s0': 1.2172137537963867,
            'right_s1': -0.4421699616027832,
            'right_w0': 0.6803204786499024,
            'right_w1': 1.432738054248047,
            'right_w2': -0.120800986907959}

        # joint angles for position above the cube
        self.above_cube_pickup = {
            'right_e0': -0.33402431618041994,
            'right_e1': 0.832568071673584,
            'right_s0': 1.0538448000732423,
            'right_s1': -0.6876068873840332,
            'right_w0': 0.26461168560791015,
            'right_w1': 1.4638011651672365,
            'right_w2': -0.016490293450927736}

        # joint angles for right limb central and cube positions
        self.right_centred = {
            'right_e0': -0.6369855214416504,
            'right_e1': 2.139136206262207,
            'right_s0': 0.8716845817199708,
            'right_s1': -0.9875001310729982,
            'right_w0': 1.3077186201782227,
            'right_w1': 2.089665325909424,
            'right_w2': -1.1757962725708009}

        self.right_cube = {
            'right_e0': -0.11926700612182618,
            'right_e1': 2.259937193170166,
            'right_s0': 0.607839886505127,
            'right_s1': -1.145883647241211,
            'right_w0': 1.36485940446167,
            'right_w1': 1.746053629815674,
            'right_w2': -1.1727283109985351}

        # joint angles for left limb central and cube positions
        self.left_centred = {
            'left_e0': -0.33785926814575196,
            'left_e1': 2.086213869140625,
            'left_s0': -0.10584467424316407,
            'left_s1': -1.0837574254028322,
            'left_w0': -1.125174906628418,
            'left_w1': 1.6751070184570314,
            'left_w2': -0.645422415765381}

        self.left_cube = {}

        # joint angles for left limb face manipulation position
        self.left_face_rotation = {
            'left_e0': -0.33172334500122075,
            'left_e1': 2.148340090979004,
            'left_s0': -0.2546408104980469,
            'left_s1': -1.1201894690734864,
            'left_w0': -1.2275681241027834,
            'left_w1': 1.601092445526123,
            'left_w2': -0.645422415765381}

        # angles for left arm rotating faces
        self.joint_left_face_rotation = {}
        # angles for left arm quater cube turn flat
        self.joint_left_cube_flat_rotation = {}
        # angles for left arm quater turn up
        self.joint_left_cube_up_rotation = {}
        # angles for left are quater turn down
        self.joint_left_cube_down_rotation = {}
        # left arm position for rotating the cube up/down
        self.joint_left_arm_down = {
            'right_e0': -0.6369855214416504,
            'right_e1': 2.139136206262207,
            'right_s0': 0.8716845817199708,
            'right_s1': -0.9875001310729982,
            'right_w0': 1.3077186201782227,
            'right_w1': 2.089665325909424,
            'right_w2': -1.1757962725708009}

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

    def perform_manouver(self, previous_manouver, current_manouver):
        """
        Performs the manouver sent to it.
        previous_manouver: the manouver that was just performed
        current_manouver: the manouver that is to be performed next
        """
        # rotate the cube to show the correct face
        if self.rotate_cube(previous_manouver[0], current_manouver[0]) is False:
            logger.error('error durring cube rotation')
            return False
        # move the left limb into position
        self.limb_left.move_to_joint_positions(left_face_rotation)
        # rotate the face
        if self.rotate_face(current_manouver) is False:
            logger.error('error durring face rotation')
            return False
        # move the left limb away
        self.left_centred['left_w2'] = self.limb_left.joint_angle('left_w2')
        self.limb_left.move_to_joint_positions(self.left_centred)
        rospy.sleep(0.5)
        self.left_centred['left_w2'] = -0.645422415765381
        self.limb_left.move_to_joint_positions(self.left_centred)
        rospy.sleep(0.5)
        logger.debug('manouver {} performed'.format(current_manouver))

    def rotate_face(self, rotation):
        """
        Rotates the left gripper to move the face the ammount specified
        rotation: the angle to rotate the face:
        {face} = 90 degrees clockwise
        {face}' = 90 degrees anticlockwise
        {face}2 = 180 degrees clockwise
        example: rotation = "F2" is rotate the face 90 degrees clockwise
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting rotate_gripper')
            return
        joint_angles = self.limb_left.joint_angles()
        if len(rotation) == 1:
            joint_angles['left_w2'] += math.pi / 2
        elif rotation[1] == "'":
            joint_angles['left_w2'] -= math.pi / 2
        elif rotation[1] == '2':
            joint_angles['left_w2'] += math.pi
        else:
            logger.error('invalid gripper rotation')
            return False
        logger.debug('moving to {}'.format(joint_angles))
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(joint_angles)
        rospy.sleep(0.5)
        self.gripper_left.open()
        rospy.sleep(0.5)

    def rotate_cube(self, current_face, next_face):
        """
        Rotates the cube so the required face is avaliable
        current_face = the current face that is in position
        next_face = the next face that needs to be in position
        returns False on error
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting rotate_cube')
            return
        # face relationships asuming a flat spin of the cube
        face_relations = {'quaterturnCW': ['FR', 'RB', 'BL', 'LF'],
                          'quaterturnACW': ['FL', 'LB', 'BR', 'RF'],
                          'halfturnCW': ['FB', 'RL', 'BF', 'LR']}
        moves_combined = current_face[0] + next_face[0]
        if moves_combined in face_relations['quaterturnCW']:
            # rotate the cube flat clockwise
            # move close to the cube
            rospy.sleep(0.5)
            # close the left gripper
            self.gripper_left.close()
            rospy.sleep(0.5)
            # open the right gripper
            self.gripper_right.open()
            rospy.sleep(0.5)
            # rotate the cube a quater turn clockwise
            rospy.sleep(0.5)
            # close the right gripper
            self.gripper_right.close()
            rospy.sleep(0.5)
            # open the left gripper
            self.gripper_left.open()
            rospy.sleep(0.5)

        elif moves_combined in face_relations['quaterturnACW']:
            # rotate the cube flat anticlockwise
            # move close to the cube
            rospy.sleep(0.5)
            # close the left gripper
            self.gripper_left.close()
            rospy.sleep(0.5)
            # open the right gripper
            self.gripper_right.open()
            rospy.sleep(0.5)
            # rotate the cube a quater turn anticlockwise
            rospy.sleep(0.5)
            # close the right gripper
            self.gripper_right.close()
            rospy.sleep(0.5)
            # open the left gripper
            self.gripper_left.open()
            rospy.sleep(0.5)

        elif moves_combined in face_relations['halfturnCW']:
            # rotate the cube flat 180 clockwise
            # move close to the cube
            rospy.sleep(0.5)
            # close the left gripper
            self.gripper_left.close()
            rospy.sleep(0.5)
            # open the right gripper
            self.gripper_right.open()
            rospy.sleep(0.5)
            # rotate the cube quater turn clockwise
            rospy.sleep(0.5)
            # close right gripper
            self.gripper_right.open()
            rospy.sleep(0.5)
            # open left gripper
            self.gripper_left.open()
            rospy.sleep(0.5)
            # return left arm to a central position
            # move left arm close into gripper range
            # close left gripper
            self.gripper_left.close()
            rospy.sleep(0.5)
            # open right gripper
            self.gripper_right.open()
            rospy.sleep(0.5)
            # move cube quater turn clockwise
            rospy.sleep(0.5)
            self.gripper_right.close()
            rospy.sleep(0.5)
            self.gripper_left.open()
            # return left arm to central position
            rospy.sleep(0.5)

        elif moves_combined[1] == 'U':
            # rotate the cube down, perform operation, then rotate back to flat
            # either rotate the cube, or move the arm so its accessable
        elif moves_combined[1] == 'D':
            # rotate the cube up, perform the operation, then rotate back to flat
        else:
            logger.error('combined previous and current moves not valid')
            return False

    def pickup_cube(self):
        """
        Picks up the cube from the preset position
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting pickup_cube')
            return
        logger.info('picking up the rubiks cube')
        self.limb_right.move_to_joint_positions(self.right_centred)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.above_cube_pickup)
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.cube_pickup)
        rospy.sleep(0.5)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.above_cube_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_centred)
        rospy.sleep(0.5)
        logger.debug('pick up cube ended')

    def putdown_cube(self):
        """
        Places the cube back after its finished
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting putdown_cube')
            return
        logger.info('putting down the rubiks cube')
        self.limb_right.move_to_joint_positions(self.angles_above_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.angles_pickup)
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.angles_above_pickup)
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.angles_centered)
        rospy.sleep(0.5)
        logger.debug('putdown cube ended')


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
