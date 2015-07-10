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

logger = logging.getLogger(__name__)
logging.getLogger('requests').setLevel(logging.WARNING)


class BaxterRubiks(object):
    """
    Solves a rubiks cube using the Baxter research robot
    and Cube Explorer
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

    def check_solver_status(self):
        """
        checks whether the solver is running
        """
        processes = commands.getoutput('ps-A')
        if 'cube512htm' in processes:
            return True
        else:
            return False

    def run_solver(self):
        """
        runs Cube Explorer if it is not already running
        """
        if self.check_solver_status() is False:
            solver = subprocess.Popen('solver/cube512htm.exe')
            logger.info('Cube Explorer started')
            return solver
        else:
            logger.info('Cube Explorer already running')
            return True

    def exit_solver(self, program):
        """
        exits Cube Explorer
        """
        if program is True:
            logger.info('Cube Explorer was initially open, leaving open')
        else:
            close_cube_explorer = raw_input('Close Cube Explorer? (Y/N)')
            if close_cube_explorer == 'y':
                logger.info('Now exiting')
                subprocess.Popen.kill(program)
                subprocess.Popen.wait(program)
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
        notation. Returns the manouvers as a list with html encoding removed.
        """
        # get the status of the connection to the webserver
        solver_connection_status = self.check_solver_connection()
        if solver_connection_status is True:
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

    def convert_to_singmaster(self, cube_colour_encoding):  # needs changing when the cube_colour_encoding is ordered properly
        """
        Convers face colour encoding to singmaster notation
        Outputs as a string in the order required for Cube Explorer:
        up - right - front - down - left - back
        """
        # reorders cube_colour_encoding ----------------------- maybe save in propper order first time around!?!?!?
        # this asumes the ordering is front - back - left - right - up - down
        # just remove the following statement when cube_colour_encoding is ordered properly.....
        required_order = [2, 5, 0, 3, 4, 1]
        cube_colour_encoding = [cube_colour_encoding[i] for i in required_order]
        # get the centre colour of each face
        front_colour = cube_colour_encoding[0][4]
        back_colour = cube_colour_encoding[1][4]
        up_colour = cube_colour_encoding[2][4]
        down_colour = cube_colour_encoding[3][4]
        left_colour = cube_colour_encoding[4][4]
        right_colour = cube_colour_encoding[5][4]
        # combines face colour strings into one string
        cube_colour_encoding_joined = ''.join(cube_colour_encoding)
        # replaces colour letter with its corresponding face letter
        cube_centre_colours = ['f', 'b', 'u', 'd', 'l', 'r']
        for colour in cube_colour_encoding_joined:
            if colour in
        cube_colour_encoding_singmaster = cube_colour_encoding_joined.replace(front_colour, 'f').replace(back_colour, 'b').replace(up_colour, 'u').replace(down_colour, 'd').replace(left_colour, 'l').replace(right_colour, 'r')

# classes : control and cube explorer, vision system, manipulations, a class to act as variable storage for each face
# (create an instance for each face required)
# --------------------------------------------------------------------------------
# control and cube explorer: main control class, contains setup_logger, check_solver_status, run_solver,
#                            exit_solver, check_solver_connection, send_solver_face_encoding, convert_to_singmaster
# --------------------------------------------------------------------------------
# vision system: get images from camera, general vision system manipulations
# --------------------------------------------------------------------------------
# manipulations: pickup cube, scan faces (one for each face?), face manipulations, place cube back down
# --------------------------------------------------------------------------------
# face class: contains variables for each face cubie value (on instance per face)
# --------------------------------------------------------------------------------


class VisionSystem(object):
    """
    Analyses images of cube faces and returns colour values
    """


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
