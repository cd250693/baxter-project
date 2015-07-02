# -*- coding: utf-8 -*-
import subprocess
import commands
import urllib2
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt

###############################################################################
# Change Log:
#    28/06/2015:
#        -Began writing the change log
#    30/06/2015:
#        -Added .git for the project, all further changes are done through that.

# try to change over to requests instead of urllib2, its much nicer
###############################################################################


# Checks if Cube Solver is running
def checkStatus():
    output = commands.getoutput('ps -A')
    if 'cube512htm' in output:
        return True
    else:
        return False


# Kills and waits for prog to end
def exitSolver(prog):
    print('Now Exiting:'),
    if prog is True:
        print('Cube Explorer was initally open, leaving open')
    else:
        closeCubeExplore = raw_input('Close Cube Explorer?(Y/N)')
        if closeCubeExplore == 'y':
            print('Now Exiting')
            subprocess.Popen.kill(prog)
            subprocess.Popen.wait(prog)
        elif closeCubeExplore == 'n':
            print('Leaving Open')
        else:
            print('Invalid Input, Now Exiting')
    return


# Image analysis, returns colour values
def visionSystem():
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
    ## get all of the faces, each one needs to call getCamImage
    #print('Show FRONT:'),
    front = getCamImage()
    #print('Done')
    #print('Show BACK:'),
    back = getCamImage()
    #print('Done')
    #print('Show UP:'),
    up = getCamImage()
    #print('Done')
    #print('Show DOWN:'),
    down = getCamImage()
    #print('Done')
    #print('Show LEFT:'),
    left = getCamImage()
    #print('Done')
    #print('Show RIGHT:'),
    right = getCamImage()
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


# Gets the image from the connected camera, passes back as a mat in RGB
# NOTE: Camera needs to be mounted as a webcam
def getCamImage():
    cap = cv2.VideoCapture(0)
    i = 0
    while i < 10:
        ret, frame = cap.read()
        time.sleep(0.001)
        i += 1
    facemat = frame[:, :, ::-1]
    #When everything done, release the capture
    cap.release()
    return facemat


# Tests to help check if the vision system worked correctly
def basicTests(facecode):
    # assigns each list entry to its corresponding face variable
    f = facecode[0]
    b = facecode[1]
    u = facecode[2]
    d = facecode[3]
    l = facecode[4]
    r = facecode[5]
    # test if each face string has 9 colours
    if any(len(x) is not 9 for x in (f, b, u, d, l, r)):
        print('not nine colours in each face')
        return False
    # test if there is 9 of each colour
    colourtest = f + b + u + d + l + r
    numO = colourtest.count('o')
    numN = colourtest.count('n')
    numR = colourtest.count('r')
    numG = colourtest.count('g')
    numW = colourtest.count('w')
    numY = colourtest.count('y')
    if any(x is not 9 for x in (numO, numN, numR, numG, numW, numY)):
        print('not nine of each colour')
        return False
    # test if each centre colour is unique
    centres = f[4] + b[4] + u[4] + d[4] + l[4] + r[4]
    cenO = centres.count('o')
    cenN = centres.count('n')
    cenR = centres.count('r')
    cenG = centres.count('g')
    cenW = centres.count('w')
    cenY = centres.count('y')
    if any(x is not 1 for x in (cenO, cenN, cenR, cenG, cenW, cenY)):
        print('centre colours are invalid')
        return False
    #all tests passed
    return True


# Converts to Singmaster notation and orders the faces correctly
def toSingmaster(facevs):
    front = facevs[0]
    back = facevs[1]
    up = facevs[2]
    down = facevs[3]
    left = facevs[4]
    right = facevs[5]
    # gets the center colour of each face
    f = front[4]
    b = back[4]
    u = up[4]
    d = down[4]
    l = left[4]
    r = right[4]
    # combines face colours in required order for the solver
    faceSing = up + right + front + down + left + back
    # replaces colour letter with its corresponding face letter
    faceSing = faceSing.replace(f, 'f').replace(b, 'b').replace(u, 'u')
    faceSing = faceSing.replace(d, 'd').replace(l, 'l').replace(r, 'r')
    return faceSing


# Sends face encoding to Cube Explorer
def sendFaceCoding(faceString):
    # Define loop to allow a timeout
    loop = 6
    # Try send data to Cube Explorer
    while loop > 0:
        try:
            # save data from sending to localhost 127.0.0.1 port 8081
            data = urllib2.urlopen('http://127.0.0.1:8081/?' + faceString)
            data = data.read()
            break
        except urllib2.URLError:
            loop -= 1
        if loop == 5:
            print('waiting'),
        else:
            print('.'),
            time.sleep(5)
    else:
        print ('couldnt connect')
        return False
    # remove HTML encodings, carrige returns and new line pointers
    data = data.replace('<HTML><BODY>\r\n', '')
    data = data.replace('\r\n</BODY></HTML>\r\n', '')
    # clears Cube Explorer's main window
    urllib2.urlopen('http://127.0.0.1:8081/?' + 'clear')
    # Returns the string as a list of manouvers
    if 'Cube cannot be solved.' in data:
        print(data)
        return False
    elif data is str:
        print(data)
        return False
    return data.split()


########## MAIN ##########
def main():
    # check if Cube Explorer is running or not
    print('Checking status:'),
    status = checkStatus()
    if status is False:
        print('Not started')
        print('Starting solver:'),
        # Runs Cube Explorer
        solver = subprocess.Popen('Solver/cube512htm.exe')
        print('Done!')
    else:
        print('Already started')
        solver = True

    # Check connection to Cube Explorer
    print('Checking connection:'),
    # Define loop variable to allow timeout functionality
    loop = 6
    while loop > 0:
        try:
            urllib2.urlopen('http://127.0.0.1:8081/?status')
            print('Sucessful')
            break
        except urllib2.URLError:
            if loop == 5:
                print('Waiting'),
            else:
                print('.'),
            time.sleep(5)
    else:
        print('Couldnt connect')
        exitSolver(solver)
        return False

    # Gets face colours from vision system
    print('Getting face information:'),
    facevs = visionSystem()
    # Check for error
    if facevs is False:
        exitSolver(solver)
        return False
    print('Done!')

    # Perform checks on the colour values from the vision system
    print('Performing checks:'),
    # Checks for errors returned from basicTests
    if basicTests(facevs) is False:
        exitSolver(solver)
        return False
    else:
        facecheck = facevs
        print('Done!')

    # convert colour values to SingMaster notation
    # additionally a single string is formed in the required order
    print('Converting to SingMaster notation:'),
    faceSingString = toSingmaster(facecheck)
    print('Done!')

    # sends face encoding to Cube Explorer
    print('Sending face encoding:'),
    man = sendFaceCoding(faceSingString)
    # check if any errors occured durring sending
    if man is True:
        print('Done!')
        print(man)
    else:
        exitSolver(solver)
        return False

    # psudoing needed here

    # exits solver
    exitSolver(solver)
    return True


# Function to allow tests of an individual function without running main
def debugger():
    # list all functions, take respective input to run
    print('#####COMMANDS#####')
    print('1 - checkStatus')
    print('2 - exitSolver')
    print('3 - visionSystem')
    print('4 - getCamImage')
    print('5 - basicTests')
    print('6 - toSingmaster')
    print('7 - sendFaceCoding')
    print('0 - Exit Debugger')
    # get user input, set as variable command
    command = raw_input('Enter a command: ')
    # ensures input is valid
    while(0 <= command <= 7):
        print('invalid input')
        command = raw_input('Enter a command: ')

    if command == 1:  # checkStatus
        pass
        return

    elif command == 2:  # exitSolver
        pass
        return

    elif command == 3:  # visionSystem
        #print('Show FRONT:'),
        front = getCamImage()
        #print('Done')
        #print('Show BACK:'),
        back = getCamImage()
        #print('Done')
        #print('Show UP:'),
        up = getCamImage()
        #print('Done')
        #print('Show DOWN:'),
        down = getCamImage()
        #print('Done')
        #print('Show LEFT:'),
        left = getCamImage()
        #print('Done')
        #print('Show RIGHT:'),
        right = getCamImage()
        #print('Done')
        # setup a matplot with all the faces
        # arranged like the cube is folded out
        titles = ['Front', 'Back', 'Up', 'Down', 'Left', 'Right']
        images = [front, back, up, down, left, right]
        position = [5, 3, 2, 8, 4, 6]
        for i in range(6):
            plt.subplot(3, 3, position[i]), plt.imshow(images[i])
            plt.title(titles[i])
            plt.xticks([]), plt.yticks([])
        plt.show()
        return

    elif command == 4:  # getCamImage
        pass
        return

    elif command == 5:  # basicTests
        pass
        return

    elif command == 6:  # toSingmaster
        pass
        return

    elif command == 7:  # sendFaceCoding
        pass
        return

    else:  # Exit Debugger
        print('Exiting Debugger')
        return


# simply runs the main function. This allows main control code to be within
# a function, reducing the risks of using the same variable names
if __name__ == '__main__':
    rundbger = raw_input('Run Debugger?(Y/N)')
    if rundbger == 'y':
        # allows easy of running programs for testing
        debugger()
        print('Exiting')
    elif rundbger == 'n':
        # beginning of program
        print('-----START-----')
        result = main()
        if result is True:
            # end of program
            print('-----FINISH-----')
        else:
        # error occured durring execution
            print('-----ERROR-----')
    else:
        print('Invalid input, Exiting')
