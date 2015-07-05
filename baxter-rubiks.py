# -*- coding: utf-8 -*-
import subprocess
import commands
import time
import cv2
import numpy as np
import requests
import argparse
from matplotlib import pyplot as plt


class BaxterRubiks(object):
    """
    Solves a rubiks cube using the Baxter research robot
    and Cube Explorer
    """

    def __init__(self):
        """
        initializes object"""
        pass

    def run_solver(program):
        """
        runs program
        """
        pass

    def exit_solver(program):
        """
        exit program
        """
        pass

    def check_solver_status():
        """
        checks whether the solver is running
        """
        pass


def main():
    """
    Creates a BaxterRubiks object and runs the control function
    """
    cube_solve = BaxterRubiks()
    cube_solve  # need to connect this to control function

if __name__ == '__main__':
    main()