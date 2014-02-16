#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32
from scitos_msgs.msg import BatteryState, MotorStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from threading import Lock
import json
import subprocess

#TODO: Write a generic topic=>JSON converter. Some related code is already inside data_centre.

def upload_summary_scp(upload_path, server, user, password, jsonfile):
    call = subprocess.call(["sshpass", "-p", password, "scp", jsonfile, "%s@%s:%s"%(user,server,upload_path) ])
    if call != 0:
        raise Exception("Failed to upload summary. Bad call to sshpass...scp")

class Sumariser(object):
    def __init__(self):

        self._create_lockable_members(['mileage',
                                       'battery',
                                       'charging',
                                       'pose',
                                       'bumper'])

        # set up subscribers
        for s in dir(self):
            if s.endswith("_cb"):
                rospy.Subscriber(getattr(self,s).sub[0],
                                 getattr(self,s).sub[1],
                                 getattr(self,s) )


    def _create_lockable_members(self, members):
        self._members = members
        for i in members:
            setattr(self,"_"+i,None)
            setattr(self,"_" + i + "_lock", Lock())
                
    def _set(self, p, v):
        lock = getattr(self,"_"+p+"_lock")
        lock.acquire()
        setattr(self,"_"+p, v)
        lock.release()

    def _get(self, p):
        lock = getattr(self,"_"+p+"_lock")
        lock.acquire()
        v = getattr(self,"_"+p)
        lock.release()
        return v
        

    def create_json_summary(self):
        summary = {}
        for i in self._members:
            summary[i]=self._get(i)
        return json.dumps(summary)

    def mileage_cb(self, data):
        self._set("mileage",data.data)
    mileage_cb.sub = ("/mileage", Float32)

    def battery_cb(self,data):
        self._set("battery",data.lifePercent)
        self._set("charging", data.charging)
    battery_cb.sub = ("/battery_state", BatteryState)

    def pose_cb(self, data):
        pose = {'position': {'x':data.pose.pose.position.x,
                                  'y':data.pose.pose.position.y,
                                  'z':data.pose.pose.position.z },
                     'orientation':{'x': data.pose.pose.orientation.x,
                                    'y': data.pose.pose.orientation.y,
                                    'z': data.pose.pose.orientation.z,
                                    'w': data.pose.pose.orientation.w}
                     }
        self._set("pose", pose)
    pose_cb.sub = ("/amcl_pose", PoseWithCovarianceStamped)

    def bumper_cb(self, data):
        self._set("bumper", data.bumper_pressed)
    bumper_cb.sub = ("/motor_status", MotorStatus)

    
if __name__ == '__main__':
    rospy.init_node("robotstate_webpublisher")
    time_delay =  rospy.get_param("~time-between",  30)
    temp_file =  rospy.get_param("~temp_file",  "/tmp/robotstate.json")
    server_url =  rospy.get_param("~server_url",  "--")
    password =  rospy.get_param("~password", "!!")
        
    if server_url !=  "--":
        # split url
        # ssh://user@host/path
        # crapppy passing :-|
        a =  server_url.find("@")
        user =  server_url[6:a]
        b =  server_url.find(":", a)
        server =  server_url[a+1:b]
        upload_path =  server_url[b+1:]
        
    s = Sumariser()
    while not rospy.is_shutdown():
        rospy.sleep(time_delay)
        
        json_string = s.create_json_summary()
        print "Written file"
        with open(temp_file,  "w") as f:
            f.write(json_string )
    
        if server_url !=  "--":
            upload_summary_scp(upload_path,  server,  user,  password,  temp_file)
            print "Uploaded"
        
    rospy.spin()
