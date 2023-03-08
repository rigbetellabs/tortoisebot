#! /usr/bin/env python3

import os
import signal
import rospy
import threading
import json 
import subprocess

from flask import Flask, request, make_response

from std_msgs.msg import UInt32, String

# def ros_callback(msg):
#     print(msg)

threading.Thread(target=lambda: rospy.init_node('rtsense_api', disable_signals=True, anonymous=True)).start()
api_status = rospy.Publisher("/api_status", String, queue_size=12)
# rospy.Subscriber('/talker', UInt32, ros_callback)
pub_status = rospy.Publisher('/pub_status', String, queue_size=10)

app = Flask(__name__)

slamLaunchSubprocess = None
poseNodeSubprocess = None
server_bringupLaunchSubprocess = None
navloadSubprocess = None
path = '/home/shubhu/catkin_ws/src/tortoisebot/tortoisebot_navigation/maps/'

def kill(pid):
    os.kill(pid,signal.SIGTERM ) #SIGKILL , SIGTERM


# startSlam
@app.route('/startslam',methods=['GET'])
def startSlam():
    global slamLaunchSubprocess
    global poseNodeSubprocess
    global server_bringupLaunchSubprocess

    server_bringupLaunchSubprocess= subprocess.Popen([
        "roslaunch", 
        "tortoisebot_firmware", 
        "server_bringup.launch"
        ])
    
    poseNodeSubprocess = subprocess.Popen([
        "rosrun", 
        "riotu_robot_pose_publisher", 
        "riotu_robot_pose_publisher"
        ])
    
    slamLaunchSubprocess = subprocess.Popen([
        "roslaunch", 
        "tortoisebot_slam", 
        "tortoisebot_slam.launch"
        ])
    
    response = make_response("Success in launching SERVER, POSE, and SLAM")
    response.status_code = 200
    api_status.publish(str(response.data))

    return response

#logs
@app.route('/statuslogs')
def log():
        if(slamLaunchSubprocess):
            print('slam'+ slamLaunchSubprocess.pid)
        if(poseNodeSubprocess):
            print('pose'+ poseNodeSubprocess.pid)
        if(server_bringupLaunchSubprocess):
            print('serverbringup'+ server_bringupLaunchSubprocess.pid)

# closing Slam
@app.route('/stopslam', methods=['GET'])
def killslam():
    global slamLaunchSubprocess
    if(server_bringupLaunchSubprocess):
        kill(slamLaunchSubprocess.pid)
        killpose()
        killserver()
        
        response = make_response("Success in killing SERVER, POSE, and SLAM")
        response.status_code = 200
        api_status.publish(str(response.data))
        
    else:
        response = make_response("Executed but the process does not exists for SERVER, POSE and SLAM")
        response.status_code = 300
        api_status.publish(str(response.data))
        

    return response 

# closing Pose        
@app.route('/stoppose')
def killpose():
    global poseNodeSubprocess
    if(poseNodeSubprocess):
        print(poseNodeSubprocess.pid)
        kill(poseNodeSubprocess.pid)
        response = make_response("Success in killing SERVER, POSE, and SLAM")
        response.status_code = 200
        api_status.publish(str(response.data))
    else:
        
        response = make_response("Executed but the process does not exists")
        response.status_code = 300
        api_status.publish(str(response.data))

    return response


# closing ServerBringup
@app.route('/stopserver')
def killserver():
    global server_bringupLaunchSubprocess
    if(server_bringupLaunchSubprocess):
        kill(server_bringupLaunchSubprocess.pid)
        response = make_response("Success in killing SERVER, POSE, and SLAM")
        response.status_code = 200
        api_status.publish(str(response.data))
    else:
        
        response = make_response("Executed but the process does not exists")
        response.status_code = 300
        api_status.publish(str(response.data))


# map saving

@app.route('/mapsaver', methods=['POST'])
def mapSaver():
    
    if request.method == 'POST':
        map_name = request.form['map_name']
        print(map_name)
        map_saver = subprocess.Popen([
            "rosrun", 
            "map_server",
            "map_saver",
            "-f",
            "{}{}".format(path,map_name)
        ])
        msg = map_name
        pub_status.publish("MapName is  :" + msg)
        

        response = make_response(" Result of map saving with map name" + map_name)
        response.status_code = 200
        api_status.publish(str(response.data))
    else:
        response = make_response("Please check map, Somethings went wrong the passed name is : " + map_name)
        response.status_code = 300
        api_status.publish(str(response.data))

    return response
# loading Nav

@app.route("/navload",methods=['POST'])
def navload():
    global navloadSubprocess  
    global poseNodeSubprocess
    if request.method == 'POST': 
        global server_bringupLaunchSubprocess

        server_bringupLaunchSubprocess= subprocess.Popen([
        "roslaunch", 
        "tortoisebot_firmware", 
        "server_bringup.launch"
        ])
        # print(path)
        # print(map_name) 
        poseNodeSubprocess = subprocess.Popen([
        "rosrun", 
        "riotu_robot_pose_publisher", 
        "riotu_robot_pose_publisher"
        ])
        
        map_name = request.form['map_name']
        navloadSubprocess = subprocess.Popen([
                                    "roslaunch",
                                    "tortoisebot_navigation",
                                    "tortoisebot_navigation.launch",
                                    "map_file:=".format(path,map_name)
                                ])
        

        response = make_response(" Result of nav loading with map name" + map_name)
        response.status_code = 200
        api_status.publish(str(response.data))

    else:
        response = make_response("Please check map and nav, Somethings went wrong the passed name is : " + map_name)
        response.status_code = 300
        api_status.publish(str(response.data))
    
    return response



# closingNav
@app.route('/stopnav')
def killnav():
    global navloadSubprocess
    if(navloadSubprocess):
        kill(navloadSubprocess.pid)
        killpose()
        killserver()

        response = make_response("Success in killing NAV and POSE")
        response.status_code = 200
        api_status.publish(str(response.data))
    else:
        response = make_response("Executed but the process does not exists")
        response.status_code = 300
        api_status.publish(str(response.data))

    return response 

#robot startup 
@app.route('/robotstartup', methods=['GET'])
def robotstartup():
    global robotstartupSubprocess
    robotstartupSubprocess = subprocess.Popen([
        "roslaunch",
        "tortoisebot_firmware",
        "bringup.launch"
    ],)
    status = robotstartupSubprocess
    print(stdout=subprocess.PIPE)
    response = make_response("Success in robot startup ")
    response.status_code = 200
    api_status.publish(str(response.data))
    
# TestApi
@app.route('/pub1', methods = ['GET'])
def hello_world():
    data = request.form
    print(request.form['mobile'])

    
    # print(data[0])
    
    
    # msg = UInt32()
    # if(slamLaunchSubprocess):
    #     msg.data = slamLaunchSubprocess.pid
    #     kill(slamLaunchSubprocess.pid)
    #     pub.publish(msg)

    # return pub.publish(msg)

if __name__ == '__main__':
    app.run(debug=True, host="192.168.0.160", port=3001)