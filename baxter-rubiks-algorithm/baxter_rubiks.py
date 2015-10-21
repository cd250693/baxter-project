# -*- coding: utf-8 -*-
import subprocess
import commands
import time
import cv2
import requests
import argparse
import logging
import math
import rospy
import baxter_interface
from sensor_msgs.msg import Image
import cv_bridge

logger = logging.getLogger('baxterRubiks')
logging.getLogger('requests').setLevel(logging.WARNING)
logging.getLogger('winediag').setLevel(logging.CRITICAL)
logging.getLogger('fixme').setLevel(logging.CRITICAL)


class BaxterRubiks(object):
    """
    Solves a rubiks cube using the Baxter Research Robot's servo system
    along with the solving algorithm Cube Explorer
    This class specifically controls the overall process and creates instances of
    the other classes.
    """

    def __init__(self, loglevel):
        """
        initializes object
        """
        self.cube_solver = CubeExplorer()
        self.baxter = Baxter()
        self.change_logger_level(loglevel)
        logger.debug('instances created')

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
        # display the image on baxters face screen
        self.baxter.display_image()
        # run cube solver if its not already running
        self.cube_solver.run_solver()
        # check the connection to Cube Explorer
        if not self.cube_solver.send_command_webserver('status', 12):
            return
        logger.info('Fill in the cube colours and solve the cube in the program')
        # get the cube values from the program
        get_solution = raw_input(
            logger.info('Enter "y" when the cube is solved, or "n" to exit'))
        if get_solution == 'y':
            manoeuvres = self.cube_solver.send_command_webserver('getLast', 12)
        elif get_solution == 'n':
            logger.info('Exiting')
            return
        else:
            logger.error('Invalid input, now exiting')
            return
        logger.debug('manoeuvres: {}'.format(manoeuvres))
        # pick up the rubiks cube
        self.baxter.pickup_cube()
        # perform each manipulation
        for manoeuvre in manoeuvres:
            if self.baxter.perform_manoeuvre(manoeuvre) is False:
                logger.error('An error occured durring manoeuvre performing')
                return
        # place down rubiks cube
        self.baxter.putdown_cube()
        # cleanup
        self.cube_solver.exit_solver()
        logger.info('-----END-----')


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
        processes = commands.getoutput('ps -A')
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
                return
            elif close_cube_explorer == 'n':
                logger.info('Leaving open')
                return
            else:
                logger.info('Invalid input, now exiting')
                return

    def send_command_webserver(self, command, retries):
        """
        Sends the command to the webserver of the cube solver
        command: the text command to send
        retries: the number of retries with 2.5 seconds inbetween
        """
        logger.debug('Sending {} to solver webserver'.format(command))
        # Loop until the retries limit is reached
        loopnum = 0
        while loopnum <= retries:
            try:
                # send command to webserver
                response = requests.get('http://127.0.0.1:8081/?' + command)
                logger.debug('{} command sent'.format(command))
                break
            except requests.exceptions.ConnectionError:
                time.sleep(2.5)
                loopnum += 1
        else:
            logger.error('Sending timed out after {}\'s'.format((retries * 2.5)))
            return False
        # Determine what to return
        if command == 'getLast':
            manoeuvres = response.text.replace('<HTML><BODY>\r\n', '')
            manoeuvres = manoeuvres.replace('\r\n</BODY></HTML>\r\n', '')
            # check if the cube was solved correctly
            if 'Cube cannot be solved' in manoeuvres:
                logger.error('{}'.format(manoeuvres))
                return False
            else:
                # returns manoeuvres as a list
                return manoeuvres.split()
        else:
            return True


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
        logger.debug('Getting robot state')
        self.robotstate = baxter_interface.RobotEnable()
        self.initial_state = self.robotstate.state().enabled
        logger.debug('Enabling robot')
        self.robotstate.enable()

        # Image path to display on baxters face screen
        self.img_path = 'rubiks_algorithm_image.jpg'

        # Initialize cube state, will be changed to keep track of the current cube state
        self.cube_state = 'F1'

        ######
        # Joint angles for both arms
        # right: right limb joint angles
        # left: left limb joint angles
        # central: position near the cube
        # cube: position where the cube can be grabbed/manipulated
        # flat: position for performing the flat cube rotations
        # vertical: positions for porforming the up/down cube rotations
        ######

        # joint angles for right limb central and cube positions
        self.right_central = {
            'right_e0': -0.33325732578735356,
            'right_e1': 2.1889905818115234,
            'right_s0': 0.5783107563720703,
            'right_s1': -1.0921943197265627,
            'right_w0': 1.2509613310913086,
            'right_w1': 1.909665325909424,
            'right_w2': -1.1788642341430664}

        self.right_cube = {
            'right_e0': -0.11926700612182618,
            'right_e1': 2.259937193170166,
            'right_s0': 0.607839886505127,
            'right_s1': -1.145883647241211,
            'right_w0': 1.36485940446167,
            'right_w1': 1.746053629815674,
            'right_w2': -1.1727283109985351}

        # joint angles for right limb pickup and place of cube
        self.right_pickup_central = {
            'right_e0': -0.8310340908874513,
            'right_e1': 0.7685243738525391,
            'right_s0': 1.2340875424438478,
            'right_s1': -0.5276893904296875,
            'right_w0': 0.6887573729736328,
            'right_w1': 1.5393497188842775,
            'right_w2': -0.09165535197143555}

        self.right_pickup_cube = {
            'right_e0': -0.7669903930664063,
            'right_e1': 0.8007379703613282,
            'right_s0': 1.2172137537963867,
            'right_s1': -0.4421699616027832,
            'right_w0': 0.6803204786499024,
            'right_w1': 1.432738054248047,
            'right_w2': -0.120800986907959}

        # joint angles for right limb central and cube flat rotation
        self.right_flat_central = {
            'right_e0': 0.7110000943725586,
            'right_e1': 1.0618981992004395,
            'right_s0': 0.32213596508789066,
            'right_s1': -0.4640291878051758,
            'right_w0': 0.6450389205688477,
            'right_w1': 2.093883773071289,
            'right_w2': -0.5273058952331543}

        self.right_flat_cube = {
            'right_e0': 0.37927674937133793,
            'right_e1': 1.366776880444336,
            'right_s0': 0.517335020123291,
            'right_s1': -0.7014127144592286,
            'right_w0': 0.8225971965637208,
            'right_w1': 2.09158280189209,
            'right_w2': -0.7478156332397461}

        # joint angles for left limb central and cube positions
        self.left_central = {
            'left_e0': -0.33785926814575196,
            'left_e1': 2.086213869140625,
            'left_s0': -0.10584467424316407,
            'left_s1': -1.0837574254028322,
            'left_w0': -1.125174906628418,
            'left_w1': 1.6751070184570314,
            'left_w2': -0.645422415765381}

        self.left_cube = {
            'left_e0': -0.37160684544067385,
            'left_e1': 2.1586944612854007,
            'left_s0': -0.218975757220459,
            'left_s1': -1.1228739354492188,
            'left_w0': -1.242140941571045,
            'left_w1': 1.5784662289306641,
            'left_w2': -0.5894321170715332}

        # joint angles for left limb central and cube flat rotation
        self.left_flat_central = {
            'left_e0': -0.6005534777709961,
            'left_e1': 1.2371555040161133,
            'left_s0': -0.3144660611572266,
            'left_s1': -0.5832961939270019,
            'left_w0': -0.7528010707946777,
            'left_w1': 2.095417753857422,
            'left_w2': -1.0404224681945802}

        self.left_flat_cube = {
            'left_e0': -0.2523398393188477,
            'left_e1': 1.585752637664795,
            'left_s0': -0.5974855161987305,
            'left_s1': -0.8295001101013184,
            'left_w0': -0.9422476978820802,
            'left_w1': 2.014116772192383,
            'left_w2': -0.7911505904479981}

        # joint angles for left limb central and cube vertical rotation
        self.left_vertical_central = {
            'left_e0': -0.6718835843261719,
            'left_e1': 1.5439516612426758,
            'left_s0': -0.08130098166503907,
            'left_s1': 0.2323980890991211,
            'left_w0': -1.0316020786743165,
            'left_w1': 2.0946507634643554,
            'left_w2': -0.42069423059692385}

        self.left_vertical_cube = {
            'left_e0': -1.1374467529174805,
            'left_e1': 1.4461603861267092,
            'left_s0': -0.3765922829956055,
            'left_s1': 0.6120583336669922,
            'left_w0': -0.974077799194336,
            'left_w1': 2.025621628088379,
            'left_w2': -0.3305728594116211}

        # Move both arms into the centre position
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)

        # Calibrate the grippers
        self.gripper_left.calibrate()
        self.gripper_right.calibrate()

    def clean_shutdown(self):
        """
        Function for safely shutting the program down
        """
        logger.debug('Exiting clean')
        self.limb_right.move_to_joint_positions(self.right_central)
        self.limb_left.move_to_joint_positions(self.left_central)
        self.gripper_right.open()
        self.gripper_left.open()
        rospy.sleep(0.5)
        if not self.initial_state:
            logger.debug('Disabling Robot')
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

    def perform_manoeuvre(self, manoeuvre):
        """
        Performs the manoeuvre sent to it.
        manoeuvre: the manoeuvre that is to be performed next
        """
        logger.debug('performing {}, intial state is {}'.format(manoeuvre, self.cube_state))
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting perform_manoeuvre')
            return False
        # Transform table for figuring out the cube rotation to perform
        face_transform_table = {
            'F1': {'90cw': 'R1', '90acw': 'L1', '180cw': 'B1', 'up': 'D1', 'down': 'U1'},
            'F2': {'90cw': 'D2', '90acw': 'U2', '180cw': 'B4', 'up': 'L2', 'down': 'R2'},
            'F3': {'90cw': 'L3', '90acw': 'R3', '180cw': 'B3', 'up': 'U3', 'down': 'D3'},
            'F4': {'90cw': 'U4', '90acw': 'D4', '180cw': 'B2', 'up': 'R4', 'down': 'L4'},
            'B1': {'90cw': 'L1', '90acw': 'R1', '180cw': 'F1', 'up': 'D3', 'down': 'U3'},
            'B2': {'90cw': 'D4', '90acw': 'U4', '180cw': 'F4', 'up': 'R2', 'down': 'L2'},
            'B3': {'90cw': 'R3', '90acw': 'L3', '180cw': 'F3', 'up': 'U1', 'down': 'D1'},
            'B4': {'90cw': 'U2', '90acw': 'D2', '180cw': 'F2', 'up': 'L4', 'down': 'R4'},
            'L1': {'90cw': 'F1', '90acw': 'B1', '180cw': 'R1', 'up': 'D4', 'down': 'U2'},
            'L2': {'90cw': 'D1', '90acw': 'U3', '180cw': 'R4', 'up': 'B2', 'down': 'F2'},
            'L3': {'90cw': 'B3', '90acw': 'F3', '180cw': 'R3', 'up': 'U4', 'down': 'D2'},
            'L4': {'90cw': 'U1', '90acw': 'D3', '180cw': 'R2', 'up': 'F4', 'down': 'B4'},
            'R1': {'90cw': 'B1', '90acw': 'F1', '180cw': 'L1', 'up': 'D2', 'down': 'U4'},
            'R2': {'90cw': 'D3', '90acw': 'U1', '180cw': 'L4', 'up': 'F2', 'down': 'B2'},
            'R3': {'90cw': 'F3', '90acw': 'B3', '180cw': 'L3', 'up': 'U2', 'down': 'D4'},
            'R4': {'90cw': 'U3', '90acw': 'D1', '180cw': 'L2', 'up': 'B4', 'down': 'F4'},
            'U1': {'90cw': 'R2', '90acw': 'L4', '180cw': 'D3', 'up': 'F1', 'down': 'B3'},
            'U2': {'90cw': 'F2', '90acw': 'B4', '180cw': 'D2', 'up': 'L1', 'down': 'R3'},
            'U3': {'90cw': 'L2', '90acw': 'R4', '180cw': 'D1', 'up': 'B1', 'down': 'F3'},
            'U4': {'90cw': 'B2', '90acw': 'F4', '180cw': 'D4', 'up': 'R1', 'down': 'L3'},
            'D1': {'90cw': 'R4', '90acw': 'L2', '180cw': 'U3', 'up': 'B3', 'down': 'F1'},
            'D2': {'90cw': 'B4', '90acw': 'F2', '180cw': 'U2', 'up': 'L3', 'down': 'R1'},
            'D3': {'90cw': 'L4', '90acw': 'R2', '180cw': 'U1', 'up': 'F3', 'down': 'B1'},
            'D4': {'90cw': 'F4', '90acw': 'B4', '180cw': 'U4', 'up': 'R3', 'down': 'L1'}}
        # figure out how to go from the current state to the next state
        if not (self.cube_state[0] == manoeuvre[0]):
            for rotation, state in face_transform_table[self.cube_state].items():
                if state[0] == manoeuvre[0]:
                    cube_rotation = rotation
                    self.cube_state = state
                    break
            else:
                logger.error('{} is not valid'.format(manoeuvre))
                return False

            # rotate the cube to show the correct face
            getattr(self, 'rotate_cube_' + cube_rotation)()

        # move the left limb into position
        self.limb_left.move_to_joint_positions(self.left_cube)
        # rotate the face
        if self.rotate_face(manoeuvre) is False:
            logger.error('error durring face rotation')
            return False

        # move the left limb away
        self.left_central['left_w2'] = self.limb_left.joint_angle('left_w2')
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.left_central['left_w2'] = -0.645422415765381
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)

        logger.debug('manoeuvre {} performed'.format(manoeuvre))

    def rotate_face(self, rotation):
        """
        Rotates the left gripper to move the face the ammount specified
        rotation: the angle to rotate the face:
        {face} = 90 degrees clockwise
        {face}' = 90 degrees anticlockwise
        {face}2 = 180 degrees clockwise
        example: rotation = "F2" is rotate the face 90 degrees clockwise
        """
        joint_angles = self.limb_left.joint_angles()
        if len(rotation) == 1:
            joint_angles['left_w2'] += (math.pi / 2) * 1.015
        elif rotation[1] == "'":
            joint_angles['left_w2'] -= (math.pi / 2) * 1.015
        elif rotation[1] == '2':
            joint_angles['left_w2'] += math.pi
        else:
            logger.error('invalid gripper rotation')
            return False
        logger.debug('moving to {}'.format(joint_angles))
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(joint_angles)
        rospy.sleep(0.2)
        self.gripper_left.open()
        rospy.sleep(0.5)
        logger.debug('rotate face {} finished'.format(rotation))

    def rotate_cube_90cw(self):
        """
        Rotate the cube flat 90 degrees clockwise
        """
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_cube)
        rospy.sleep(0.2)
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_flat_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_flat_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_flat_cube)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_flat_cube)
        rospy.sleep(0.2)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.gripper_left.open()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(self.left_flat_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_flat_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_cube)
        rospy.sleep(0.2)
        logger.debug('rotate cube 90cw finished')
        return

    def rotate_cube_90acw(self):
        """
        Rotate the cube flat 90 degrees anticlockwise
        """
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_flat_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_flat_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_flat_cube)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_flat_cube)
        rospy.sleep(0.2)
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_flat_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_flat_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_cube)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_cube)
        rospy.sleep(0.2)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.gripper_left.open()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        logger.debug('rotate cube 90acw finished')
        return

    def rotate_cube_180cw(self):
        """
        Rotate the cube flat 180 degrees clockwise
        """
        self.rotate_cube_90cw()
        self.rotate_cube_90cw()
        logger.debug('rotate cube 180cw finished')
        return

    def rotate_cube_up(self):
        """
        Rotate the cube vertically up
        """
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_vertical_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_vertical_cube)
        rospy.sleep(0.2)
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_vertical_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_cube)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_cube)
        rospy.sleep(0.2)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.gripper_left.open()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        logger.debug('rotate cube up finished')
        return

    def rotate_cube_down(self):
        """
        Rotate the cube virtically down
        """
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_cube)
        rospy.sleep(0.2)
        self.gripper_left.close()
        rospy.sleep(0.5)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_vertical_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_vertical_cube)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_cube)
        rospy.sleep(0.2)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.gripper_left.open()
        rospy.sleep(0.5)
        self.limb_left.move_to_joint_positions(self.left_vertical_central)
        rospy.sleep(0.2)
        self.limb_left.move_to_joint_positions(self.left_central)
        rospy.sleep(0.2)
        logger.debug('rotate cube down finished')
        return

    def pickup_cube(self):
        """
        Picks up the cube from the preset position
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting pickup_cube')
            return
        logger.info('Picking up the rubiks cube')
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_pickup_central)
        rospy.sleep(0.2)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_pickup_cube)
        rospy.sleep(0.2)
        self.gripper_right.close()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_pickup_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_cube)
        rospy.sleep(0.2)
        logger.debug('pick up cube ended')

    def putdown_cube(self):
        """
        Places the cube back after its finished
        """
        if rospy.is_shutdown():
            logger.error('rospy was shutdown, exiting putdown_cube')
            return
        logger.info('Putting down the rubiks cube')
        self.limb_right.move_to_joint_positions(self.right_pickup_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_pickup_cube)
        rospy.sleep(0.2)
        self.gripper_right.open()
        rospy.sleep(0.5)
        self.limb_right.move_to_joint_positions(self.right_pickup_central)
        rospy.sleep(0.2)
        self.limb_right.move_to_joint_positions(self.right_central)
        rospy.sleep(0.2)
        logger.debug('putdown cube ended')


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
