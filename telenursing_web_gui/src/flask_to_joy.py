#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: 10/27/2020
# Project: Autonomous Camera Assitance for Teleoperation

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
global gripperValList
global camObjList

# initalize globals
x,y,z,yaw,p,r = (0, 0, 0, 0, 0, 0)
gripperValList = [0]
camObjList = [False, False, True]

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


@app.route('/closeGripper', methods=['POST'])
def closeGripperCB():
    """
    This function serves as the API enpoint for the closeGripper button. This function
    will decrement a global variable representing the current state of the gripper by one.

    params:
        None
    returns:
        status (String)
    """
    global gripperValList 
    valDict = request.get_json()
    gripperValList[0] -= int(valDict['val'])

    # constrain gripperVal to min out at 0
    if(gripperValList[0] < 0):
        gripperValList[0] = 0
    return "OK"

@app.route('/openGripper', methods=['POST'])
def openGripperCB():
    """
    This function serves as the API endpoint for the openGripper button. This function
    will increment a global variable representing the current state of the gripper by one.

    params:
        None
    returns:
        status (String)
    """
    global gripperValList
    valDict = request.get_json()
    gripperValList[0] += int(valDict['val'])

    # constrain gripperVal to max out at 10
    if(gripperValList[0] > 10):
        gripperValList[0] = 10
    return "OK"

@app.route('/setCamObjectives', methods=['POST'])
def setCamObjectivesCB():
    """
    This function serves as the API endpoint for the camera control objective
    function HTML form. When the form is POSTed to this function, the state of the
    checkboxes are extracted, and global variables are updated accordingly.

    params:
        None
    returns:
        status (String)
    """
    global camObjList
    camObjList[0] = request.form.get('camObj1') != None
    camObjList[1] = request.form.get('camObj2') != None
    camObjList[2] = request.form.get('camObj3') != None
    return "Camera optimization objectives set. Please use the back button to resume control."

def publishLoopThread(joyPub, camObjPubList):
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
    global gripperValList
    global camObjList

    # initialize control variables
    zeroList = [0,0,0,0,0,0]
    zeroCount = 0
    lastGripperVal = 0

    while not rospy.is_shutdown():
        # construct axisList
        axisList = [x,y,z,yaw,p,r]

        # scale each element
        for i in range(len(axisList)):
            if(axisList[i] != 0):
                axisList[i] = axisList[i]/100

        # always publish axis or button values change
        if(axisList != zeroList or gripperValList[0] != lastGripperVal):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            JoyMessage.buttons = gripperValList
            joyPub.publish(JoyMessage)

            # update zeroCount
            zeroCount = 0

        # only publish three zero-valued axis values to rate limit
        # requests
        elif(axisList == zeroList and zeroCount < 3):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            JoyMessage.buttons = gripperValList
            joyPub.publish(JoyMessage)

            #update zeroCount
            zeroCount += 1

        # update lastGripperVal
        lastGripperVal = gripperValList[0]

        # publish current camera control objectives
        camObjPubList[0].publish(camObjList[0])
        camObjPubList[1].publish(camObjList[1])
        camObjPubList[2].publish(camObjList[2])

        # update delay of 200ms to match refresh rate of 200ms
        # in joystick data recieved from webpage
        time.sleep(0.3)

def main():
    # init node
    rospy.init_node('flask_to_joy')
    joyPub = rospy.Publisher('virtualJoystick', Joy, queue_size=1)

    # setup camera control objective boolean publishers
    camObjPubList = []
    camObjPubList.append(rospy.Publisher('isOcclusionAvoidance', Bool, queue_size=1))
    camObjPubList.append(rospy.Publisher('isSaliencyMaximization', Bool, queue_size=1))
    camObjPubList.append(rospy.Publisher('isInfoEntropyMaximization', Bool, queue_size=1))

    rospy.loginfo("node initalized...")

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    # start publishing thread
    thread = threading.Thread(target=publishLoopThread, args=(joyPub, camObjPubList,))
    thread.start()

if __name__ == "__main__":
    main()
