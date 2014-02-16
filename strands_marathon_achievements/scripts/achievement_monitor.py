#! /usr/bin/env python

import roslib
import rospy
import pymongo
import strands_tweets.srv
import ros_mary_tts.srv 
from  waypoint_patroller import log_util

        # self.start_time = None
        # self.start_mileage = 0
        # self.bumper_hits=[]
        # self.navigation_fails=[]
        # self.finish_time = None
        # self.start_mileage = 0
        # self.finish_mileage = 0
        # self.stamped_mileage=[]
        # self.stamped_battery=[]
        # self.stamped_charges=[]
        # self.waypoints_stamps=[]
        # self.active_point_name = ""

tweet_achievements = True
speak_achievements = False

tweet_service = None
speech_service = None

def wipe_achievement_records(achievements_db):
	achievements_db.remove()

def broadcase_achievement(type, current_value, target_value, achievement):

	achievement_text = 'Achievement Unlocked! ' + achievement
	
	if tweet_achievements:
		try:
			tweet_service(achievement_text + ' #RobotMarathon', False)
		except rospy.ServiceException as exc: 
			print("Service did not process request: " + str(exc))

	if speak_achievements:
		try:
			speech_service(achievement_text)
		except rospy.ServiceException as exc: 
			print("Service did not process request: " + str(exc))


	print achievement


def handle_achievement(achievements_db, type, current_value, target_value, achievement):

	achievement_doc = {"type": type, "val": target_value}
	existing = achievements_db.find_one(achievement_doc)
	
	# if we haven't seen this before
	if existing == None:
		broadcase_achievement(type, current_value, target_value, achievement)
		achievements_db.insert(achievement_doc)
		return True
	else:
		return False


def create_test_val(achievement_type, episode):

	test_val = None

	# translate stat into target value type
	if achievement_type == 'run_duration':
		# hours
		test_val = (latest.stamped_mileage[-1][1].total_seconds()/60)/60
	elif achievement_type == 'distance':
		# m
		test_val = latest.stamped_mileage[-1][0]
	else:
		try:
			# try to match achievement_type to a field from the episoode
			test_val = episode.__dict__[achievement_type]
			# if this is a list of things, assume we want the count instead
			if isinstance(test_val, list):
				test_val = len(test_val)
		except KeyError, e:
			rospy.loginfo("episode struct doesn't have field %s", achievement_type)	
		

	return test_val


if __name__ == '__main__':
	rospy.init_node("achievement_monitor")
	host = rospy.get_param("datacentre_host") 
	port = rospy.get_param("datacentre_port")

	gen = log_util.StatGenerator(host, port) 
	client = pymongo.MongoClient(host, port) 
	db=client.autonomous_patrolling 
	achievements_db=db["achievements"]
	
	if rospy.get_param("~wipe_achievements", False):
		rospy.loginfo("Wiping records of achievements from database")
		wipe_achievement_records(achievements_db)


	tweet_achievements = rospy.get_param("~tweet_achievements", False)
	if tweet_achievements:
		tweet_service = rospy.ServiceProxy('/strands_tweets/Tweet', strands_tweets.srv.Tweet)  

	speak_achievements = rospy.get_param("~speak_achievements", False)
	if speak_achievements:
		speech_service = rospy.ServiceProxy('/ros_mary', ros_mary_tts.srv.ros_mary)  


	rate = rospy.Rate(1./360) # in hz -- summary is only updated every 5 minutes 
	while not rospy.is_shutdown():

		# load params
		achievement_list = rospy.get_param('/achievements')		

		# get latest episode
		latest = gen.get_episode(gen.get_latest_run_name())
		print latest.get_json_summary()

		# get duration from last entry
		# print latest.stamped_mileage[-1][1]

		achievement_annouced = False

		for achievement_type in achievement_list:

			if not achievement_annouced:
				test_val = create_test_val(achievement_type, latest)
			
				achievement_targets = achievement_list[achievement_type]

				for achievement_target in achievement_targets:

					rospy.loginfo("is %s >= %s : %s", test_val, achievement_target['val'], test_val >= achievement_target['val'])
					if test_val >= achievement_target['val']:
						if handle_achievement(achievements_db, achievement_type, test_val, achievement_target['val'], achievement_target['achievement']):
							achievement_annouced = True
							break
			else:
				print "ignoring other achievements now"
					

		rate.sleep()
