# -*- coding: utf-8 -*-
import subprocess
import commands
import time
import cv2
import numpy as np
import requests
import argparse
from matplotlib import pyplot as plt
import logging
from collections import Counter

logger = logging.getLogger(__name__)
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
# Manipulations: pickup cube, scan faces (one for each face?), face manipulations, place cube back down
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
        self.loglevel = loglevel

    def setup_logger(self, loggerlevel):
        """
        Sets up the logging with the specified level
        """
        logformat = '%(asctime)s: %(message)s'
        if loggerlevel:
            logging.basicConfig(level=logging.DEBUG, format=logformat)
        else:
            logging.basicConfig(level=logging.INFO, format=logformat)

    def solve_rubiks_cube(self):
        """
        Function that automates the process of solving the rubiks cube using
        Baxter
        """
        logger.info('-----START-----')

        # create required instances of each class
        cube_solver = CubeExplorer()
        vision_system = VisionAnalysis()
        baxter_left_arm = Manipulations()
        baxter_right_arm = Manipulations()
        front_face = CubeFace()
        back_face = CubeFace()
        up_face = CubeFace()
        down_face = CubeFace()
        left_face = CubeFace()
        right_face = CubeFace()
        logger.debug('instances created')
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

        # send face images to the vision system for analysis

        # test the colours from the vision system

        # convert the cube colours into singmaster notation

        # send face encoding to Cube Explorer

        # perform each manipulation

        # place down rubiks cube
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
        solver = None

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


class Manipulations(object):
    """
    Controls Baxters arms while manipulating the cube.
    Functions will include: get the cube, place back the cube,
                            scan the cubes faces, roate the cube faces
                            rotate the cube itself, move cube to a central area
    """


class CubeFace(object):
    """
    A class for variable storage along with relevant functions:
        get the middle colour, send back a string containing all the cubelet values
    """
    def __init__(self):
        cubelet_colours = None
        face_image = None

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
                                     formatter_class='argparse.RawTextHelpFormatter')
    parser.add_argument('-v', '--verbose', action='store_true', help='increases verbosity')
    return parser.parse_args()


def main():
    """
    Gets command line arguments, creates a BaxterRubiks object and runs the control function
    """
    args = parse_arguments()
    cube_solve = BaxterRubiks(args.verbose)
    cube_solve  # need to connect this to control function

if __name__ == '__main__':
    main()
