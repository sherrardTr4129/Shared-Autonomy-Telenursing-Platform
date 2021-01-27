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
from telenursing_web_gui.srv import changeStreamState, changeStreamStateResponse

# initialize flask
app = Flask(__name__)
CORS(app)

# setup globals
global x,y,z,yaw,p,r
global fwdRev, spin
global MobilePageActive
global TaskPageActive
global primaryStreamURL
global secondaryStreamURL

# initalize globals
x,y,z,yaw,p,r = (0, 0, 0, 0, 0, 0)
fwdRev, spin = (0,0)
MobilePageActive = False
TaskPageActive = False
primaryStreamURL = "http://localhost:8080/stream_viewer?topic=/trina2_1/right_arm_cam/color/image_raw1111"
secondaryStreamURL = "http://localhost:8080/stream_viewer?topic=/trina2_1/right_arm_cam/color/image_raw"

@app.route("/setPrimaryStream", methods=['POST'])
def setPrimaryStreamCB():
    """
    This function serves as the endpoint for the setPrimaryStream API. It allows an outside requester
    to manually change the primary and secondary video stream URLs.

    params:
        None
    returns:
        None
    """
    global primaryStreamURL
    global secondaryStreamURL

    streamDict = request.get_json()
    primaryStreamURL = str(streamDict["primaryStream"])
    secondaryStreamURL = str(streamDict["secondaryStream"])

    return "OK"


@app.route("/getPrimaryStream", methods=['GET'])
def getPrimaryStreamCB():
    """
    This function serves as the endpoint for the getPrimaryStream API. It returns the text
    representation of the primary and secondary stream URLs in a JSON object to the requester.

    params:
        None
    returns:
        None
    """
    global primaryStreamURL
    global secondaryStreamURL

    jsonStr = jsonify({"primaryStream" : primaryStreamURL, "secondaryStream" : secondaryStreamURL})
    return jsonStr

@app.route("/mobilePageActive", methods=['POST'])
def mobilePageReadyCB():
    """
    This function serves as the callback to the mobile page active API endpoint.
    Calls to this endpoint are made on page load of the mobile robot control page. This
    function updates the global state of the current loaded page.

    params:
        None
    returns:
        status (string)
    """

    global MobilePageActive
    global TaskPageActive
    MobilePageActive = True
    TaskPageActive = False

    return "OK"

@app.route("/taskPageActive", methods=['POST'])
def taskPageReadyCB():
    """
    This function serves as the callback to the task page active API endpoint.
    Calls to this endpoint are made on page load of the task view and control page. This
    function updates the global state of the current loaded page.

    params:
        None
    returns:
        status (string)
    """

    global MobilePageActive
    global TaskPageActive
    MobilePageActive = False
    TaskPageActive = True

    return "OK"

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

def flipURLStreamStrings():
    """
    This function swaps the primary and secondary URL stream strings.

    params:
        None
    returns:
        None
    """

    global primaryStreamURL
    global secondaryStreamURL

    tempString = ""

    # swap the strings
    tempString = primaryStreamURL
    primaryStreamURL = secondaryStreamURL
    secondaryStreamURL = tempString

def handleSwapService(req):
    """
    This function serves as the service handler for the URL swap ROS service. When a request
    is made to this service, the stream URLs will be swapped

    params:
        req: The ROS service request message payload
    returns:
        None
    """
    if(req.changeState == True):
        flipURLStreamStrings()
    
    return changeStreamStateResponse(req.changeState)

def publishLoopThread(joyPub, mobileActivePub, taskActivePub):
    """
    This function will be instantiated as a thread. This thread will continually
    publish the global variables that are being updated in the above callbacks
    on a set interval of 200ms, and when other specific criteria are met.

    params:
        joyPub (sensor_msgs/Joy publisher obj): The instantiated Joy publisher object
        mobileActivePub (std_msgs/Bool publisher): A Bool publisher object for the 
                                                   mobile control page active status
        taskActivePub (std_msgs/Bool publisher): A Bool publisher object for the
                                                 task control page active status
    returns:
        None
    """
    # setup globals
    global x,y,z,yaw,p,r
    global fwdRev, spin
    global MobilePageActive
    global TaskPageActive

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

        # publish mobile page active status
        mobileActiveMessage = Bool()
        mobileActiveMessage.data = MobilePageActive
        mobileActivePub.publish(mobileActiveMessage)

        taskActiveMessage = Bool()
        taskActiveMessage.data = TaskPageActive
        taskActivePub.publish(taskActiveMessage)

        # update delay of 200ms to match refresh rate of 200ms
        # in joystick data recieved from webpage
        time.sleep(0.3)

def main():
    # init node
    rospy.init_node('flask_to_joy')

    # init publishers
    joyPub = rospy.Publisher('virtualJoystick', Joy, queue_size=1)
    mobileActivePub = rospy.Publisher('mobilePageActive', Bool, queue_size=1)
    taskActivePub = rospy.Publisher('taskPageActive', Bool, queue_size=1)

    # init service handlers
    serviceHandler = rospy.Service("toggleStream", changeStreamState, handleSwapService)

    rospy.loginfo("node initalized...")

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    # start publishing thread
    thread = threading.Thread(target=publishLoopThread, args=(joyPub, mobileActivePub, taskActivePub,))
    thread.start()

if __name__ == "__main__":
    main()
