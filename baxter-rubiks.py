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

    def run_solver(self, program):
        """
        runs program
        """
        pass

    def exit_solver(self, program):
        """
        exit program
        """
        pass

    def check_solver_status(self):
        """
        checks whether the solver is running
        """
        pass


def parse_arguments():
    """
    sets up argument parsing
    """
    parser = argparse.ArgumentParser(prog='baxter-rubiks-algorithm',
                                     description='Solves a rubiks cube using the baxter research robot and a rubiks solver',
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
