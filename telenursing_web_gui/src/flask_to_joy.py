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
from std_msgs.msg import Bool, String
from telenursing_web_gui.srv import changeStreamState, changeStreamStateResponse
from telenursing_web_gui.srv import homeRobotLeftArmReq

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
global primaryStreamCam
global secondaryStreamCam

# initalize globals
x,y,z,yaw,p,r = (0, 0, 0, 0, 0, 0)
fwdRev, spin = (0,0)
MobilePageActive = False
TaskPageActive = False
primaryStreamURL = "http://localhost:8080/stream_viewer?topic=/trina2_1/primaryCameraStream/color/image_raw"
secondaryStreamURL = "http://localhost:8080/stream_viewer?topic=/trina2_1/secondaryCameraStream/color/image_raw"
primaryStreamCam = "head"
secondaryStreamCam = "leftArm"

# Misc variables
homeLeftArmReq = "/trina2_1/homeLeftArmReq"

@app.route("/makeRobotHomeReq", methods=['GET'])
def makeRobotHomeReq():
    """
    this function makes a ROS service call to the robot arm controller
    manager to home the arm.

    params:
        None
    returns:
        None
    """
    rospy.wait_for_service(homeLeftArmReq)
    try:
        # setup service client
        serviceClient = rospy.ServiceProxy(homeLeftArmReq, homeRobotLeftArmReq)

        # make service call
        result = serviceClient(True)

        if(result):
            rospy.loginfo("left arm homed")
        else:
            rospy.logerr("could not home left arm!")

    except rospy.ServiceException as err:
        rospy.logerr("service call failed: %s" % err)

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

    global primaryStreamCam
    global secondaryStreamCam

    tempString = ""

    # swap the cam strings
    tempString = primaryStreamCam
    primaryStreamCam = secondaryStreamCam
    secondaryStreamCam = tempString

@app.route('/swapStreams', method=['POST'])
def swapStreamsAPI():
    """
    This is one of two functions that allows external software hooks 
    to swap the primary and secondary camera streams. This function in particular
    exposes this functionallity via the flask API. 

    params:
        None
    returns:
        None
    """
    flipURLStreamStrings()

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

def publishLoopThread(joyPub, mobileActivePub, taskActivePub, primaryStreamPub, secondaryStreamPub):
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
        primaryStreamPub (std_msgs/String publisher): A String publisher object for
                                                      the current primary camera
        secondayStreamPub (std_msgs/String publisher): A String publisher object for the
                                                       current secondary camera
    returns:
        None
    """
    # setup globals
    global x,y,z,yaw,p,r
    global fwdRev, spin
    global MobilePageActive
    global TaskPageActive
    global primaryStreamCam
    global secondaryStreamCam

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

        # publish primary and secondary stream status
        primaryStreamCamMessage = String()
        primaryStreamCamMessage.data = primaryStreamCam
        primaryStreamPub.publish(primaryStreamCamMessage)

        secondaryStreamCamMessage = String()
        secondaryStreamCamMessage.data = secondaryStreamCam
        secondaryStreamPub.publish(secondaryStreamCamMessage)

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
    primaryCamPub = rospy.Publisher('primaryCam', String, queue_size=1)
    secondaryCamPub = rospy.Publisher('secondaryCam', String, queue_size=1)
    # init service handlers
    serviceHandler = rospy.Service("toggleStream", changeStreamState, handleSwapService)

    rospy.loginfo("node initalized...")

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    # start publishing thread
    thread = threading.Thread(target=publishLoopThread, args=(joyPub, mobileActivePub, taskActivePub, primaryCamPub, secondaryCamPub,))
    thread.start()

if __name__ == "__main__":
    main()
