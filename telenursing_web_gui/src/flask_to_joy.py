#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: 02/27/2021
# Project: Socially Distanced Tele-Nursing Project

# import required libraries
import rospy
import time
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# initialize flask
app = Flask(__name__)
CORS(app)

# setup globals
global x,y,z,yaw,p,r
global fwdRev, spin

# initalize globals
x,y,z,yaw,p,r = (0, 0, 0, 0, 0, 0)
fwdRev, spin = (0,0)

@app.route('/fwdRevJoyPost', methods=['POST'])
def fwdRevJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the fwdRev
    joystick are POSTed from the frontend javascript. This function extracts
    the message payload and updates a global variable with the contents

    params:
        None
    returns:
        status (String)
    """
    global fwdRev
    frDict = request.get_json()
    fwdRev = float(frDict['FwdRev'])
    return "OK"

@app.route('/spinJoyPost', methods=['POST'])
def spinJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the spin
    joystick are POSTed from the frontend javascript. This function extracts
    the message payload and updates a global variable with the contents

    params:
        None
    returns:
        status (String)
    """
    global spin
    spinDict = request.get_json()
    spin = float(spinDict['spin'])
    return "OK"


@app.route('/xyJoyPost', methods=['POST']) 
def xyJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the xy
    joystick are POSTed from the frontend javascript. This function extracts
    the message payload and updates a global variable with the contents

    params:
        None
    returns:
        status (String)
    """
    global x,y
    xyDict = request.get_json()
    x = float(xyDict['x'])
    y = float(xyDict['y'])
    return "OK"

@app.route('/zJoyPost', methods=['POST'])
def zJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the z-axis joystick
    are POSTed to. This function extracts the message payload and updates a
    global veriable with the contents.

    params:
        None
    returns:
        status (String)
    """
    global z
    gzDict = request.get_json()
    z = float(gzDict['z'])
    return "OK"

@app.route('/yawPitchJoyPost', methods=['POST'])
def yawPitchJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the xy
    joystick are POSTed from the frontend javascript. This function extracts
    the message payload and updates a global variable with the contents

    params:
        None
    returns:
        status (String)
    """
    global yaw,p
    yawPitchDict = request.get_json()
    yaw = float(yawPitchDict['yaw'])
    p = float(yawPitchDict['pitch'])
    return "OK"

@app.route('/rollJoyPost', methods=['POST'])
def rollJoyDataCB():
    """
    This function serves as the API endpoint to which updates from the xy
    joystick are POSTed from the frontend javascript. This function extracts
    the message payload and updates a global variable with the contents

    params:
        None
    returns:
        status (String)
    """
    global r
    rollDict = request.get_json()
    r = float(rollDict['roll'])
    return "OK"


def publishLoopThread(joyPub):
    """
    This function will be instantiated as a thread. This thread will continually
    publish the global variables that are being updated in the above callbacks
    on a set interval of 200ms, and when other specific criteria are met.

    params:
        joyPub (sensor_msgs/Joy publisher obj): The instantiated Joy publisher object
        camObjPubList (std_msgs/Bool publisher obj[]): A list of instantiated Bool publisher objects

    returns:
        None
    """
    # setup globals
    global x,y,z,yaw,p,r
    global fwdRev, spin

    # initialize control variables
    zeroList = [0,0,0,0,0,0,0,0]
    zeroCount = 0

    while not rospy.is_shutdown():
        # construct axisList
        axisList = [fwdRev,spin,x,y,z,yaw,p,r]

        # scale each element
        for i in range(len(axisList)):
            if(axisList[i] != 0):
                axisList[i] = axisList[i]/100

        # always publish axis or button values change
        if(axisList != zeroList):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            joyPub.publish(JoyMessage)

            # update zeroCount
            zeroCount = 0

        # only publish three zero-valued axis values to rate limit
        # requests
        elif(axisList == zeroList and zeroCount < 3):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            joyPub.publish(JoyMessage)

            #update zeroCount
            zeroCount += 1


        # update delay of 200ms to match refresh rate of 200ms
        # in joystick data recieved from webpage
        time.sleep(0.3)

def main():
    # init node
    rospy.init_node('flask_to_joy')
    joyPub = rospy.Publisher('virtualJoystick', Joy, queue_size=1)

    rospy.loginfo("node initalized...")

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    # start publishing thread
    thread = threading.Thread(target=publishLoopThread, args=(joyPub,))
    thread.start()

if __name__ == "__main__":
    main()
