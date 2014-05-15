#! /usr/bin/env python

import roslib
import rospy
import actionlib
import math
import time
import ftp_upload.msg 

class FTPUploadServer:
  def __init__(self):
    rospy.init_node('ftp_upload_actionserver')

    self.server = actionlib.SimpleActionServer('ftp_upload', ftp_upload.msg.FtpUploadAction, self.execute, False)
    self.server.register_preempt_callback(self.preemptCallback)
    self.server.start()
    rospy.loginfo('Ftp upload server started')

    self.cancelled = False

    self.feedback = ftp_upload.msg.FtpUploadFeedback()
    self.result = ftp_upload.msg.FtpUploadResult()
    self.result.success = True


  def execute(self, goal):

	print 'Starting FTP upload to server ',goal.ftp_server

    	if self.cancelled:
                self.result.success = False
                self.server.set_preempted(self.result)
	else:
                self.result.success = True
                self.server.set_succeeded(self.result)


  def preemptCallback(self):
	self.cancelled = True
   	print 'FTP upload preempted'

if __name__ == '__main__':
  server = FTPUploadServer()
  rospy.spin()

