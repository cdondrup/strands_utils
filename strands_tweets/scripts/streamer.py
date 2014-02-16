#!/usr/bin/env python

import rospy
from time import sleep
from strands_tweets.srv import *
import rospy
from twython import Twython, TwythonError
import scitos_msgs.msg
import scitos_apps_msgs.msg

from twython import TwythonStreamer

class MyStreamer(TwythonStreamer):

    def on_success(self, data):
        if 'text' in data:
            received = data['text'].encode('utf-8')
            print received
            if '@SimLindaStrands' in received :
                request = received.replace("@SimLindaStrands", "");
                user = data['user']['screen_name']
                code = self._get_req_code(request)
                #Battery
                if code == 1 :
                    charsubs = rospy.Subscriber("/battery_state", scitos_msgs.msg.BatteryState, self._battery_callback)
                    timeout=0
                    self._battery_received=False
                    while (not self._battery_received) and timeout < 100 :
                        sleep(0.05)
                        timeout=timeout+1
                    charsubs.unregister()
                    if timeout >= 100 :
                        answer = "@%s I can\'t tell you right now, try again later" %user
                    else :
                        if self._at_charger :
                            answer = "@%s my battery level is %d, and I am charging" %(user,self._battery_level)
                        else :
                            answer = "@%s my battery level is %d" %(user,self._battery_level)

                #Position
                if code == 2 :
                    answer = "@%s You asked for my Position, I will be able to answer soon" %user
                    print answer


                if 'Status' in request:
                    answer = "@%s You asked for my Status, I will be able to answer soon" %user
                    print answer
                    understood = True
                if 'Who are you?' in request:
                    answer = "@%s I am Sim Linda Strands a Simualted Robot from the STRANDS family" %user
                    print answer
                    understood = True
                if 'Are you at Charger?' in request:
                    understood = True
                    charsubs = rospy.Subscriber("/battery_state", scitos_msgs.msg.BatteryState, self._battery_callback)
                    timeout=0
                    self._battery_received=False
                    while (not self._battery_received) and timeout < 100 :
                        sleep(0.05)
                        timeout=timeout+1
                    charsubs.unregister()
                    if timeout >= 100 :
                        answer = "@%s I can\'t tell you right now, try again later" %user
                    else :
                        if self._at_charger :
                            answer = "@%s yes after a hard patrol run I always come to my station and get some energy" %user
                        else :
                            answer = "@%s no I am working hard as usual" %user                    
                if not understood :
                    answer = "@%s I still don\'t know what you are talking about, but I will soon" %user
                    print answer
                twitter.update_status(status=answer)
                
        # Want to disconnect after the first result?
        # self.disconnect()

    def _get_req_code(self, request):
        code = -1
        if 'Battery' in request:
            code = 1
        if 'Position' in request:
            code = 2
        return code

    def on_error(self, status_code, data):
        print status_code, data

    def _battery_callback(self, data):
        print "."
        self._at_charger=data.charging
        self._battery_level=data.lifePercent
        self._battery_received=True


if __name__ == '__main__':
    rospy.init_node('tweet_replier')
    print "Init node"
    APP_KEY = rospy.get_param("/twitter/appKey")
    APP_SECRET = rospy.get_param("/twitter/appSecret")
    OAUTH_TOKEN = rospy.get_param("/twitter/oauthToken")
    OAUTH_TOKEN_SECRET = rospy.get_param("/twitter/oauthTokenSecret")
    print "Creating Twitter Object"
    twitter = Twython(APP_KEY, APP_SECRET, OAUTH_TOKEN, OAUTH_TOKEN_SECRET)
    print "Creating Streamer"
    stream = MyStreamer(APP_KEY, APP_SECRET, OAUTH_TOKEN, OAUTH_TOKEN_SECRET)
    stream.user()
    self.disconnect()
    #stream.site(follow='twitter')
