# Announcing achievements

This package contains a node which monitors the current episode of the robot's patrolling for times when it passes certain milestone points. When it hits these points it then tweets and/or speaks an announcement of this achievement.

## Twitter

If you want to tweet your announcements, you must have the `tweets.py` node running from `strands_tweets`. This is detailed in its [README](https://github.com/strands-project/strands_utils/blob/master/strands_tweets/README.md).

## Speech

If you want to speak your announcements, you must have the `ros_mary.launch` file running from `ros_mary_tts`. This is detailed in its [README](https://github.com/strands-project/strands_hri/blob/master/ros_mary_tts/README.md).

## Usage

You first need to load your target achievements into the ROS parameter server. There are a list of achievements provided for the marathon, and we should all use the same (although you are free to change the announcements associated with each one).
```
rosparam load `rospack find strands_marathon_achievements`/default-achievements.yaml
```
You can check they've loaded correctly with
```
rosparam get achievements
```

Next run the nodes you want to use to communicate to the world. If you want speech, and haven't run Mary already (e.g. it's included in the current patroller launch file) do:
```
roslaunch ros_mary_tts ros_mary.launch
```
If you want Twitter announcements do:
```
rosrun strands_tweets tweet.py
```
Finally run the monitor node which will compare the target achievements against the current patrolling episode, as provided by the [patrol run logger](https://github.com/strands-project/autonomous_patrolling/wiki/Patrol-Run-Logs):
```
rosrun strands_marathon_achievements achievement_monitor.py  _wipe_achievements:=<bool> _tweet_achievements:=<bool> _speak_achievements:=<bool>
```
The parameters are as follows:

 - `wipe_achievements`: removes all previously announced achievements from the database, causing your robot to reachieve them as if for the first time. This should really only be done at the start of the marathon week, as each robot should only achieve each achievement once.

- `tweet_achievements`: Use the twitter service to announce achievements.

- `speak_achievements`: Use the twitter service to announce achievements.


## Defining Achievements

Achievements are defined using [ROS parameters](http://wiki.ros.org/Parameter%20Server). They are defined as follows
```yaml
achievements:
  <achievemment_type>: 
  - {val: 0, achievement: "I got to 0"}
  - {val: 1, achievement: "I to to 1"}
    <achievemment_type>: 
  - {val: 1000, achievement: "I got 1000 in something else"}
```
e.g.
```yaml
achievements:
  run_duration: 
  - {val: 0, achievement: "Let's get cracking"}
  - {val: 1, achievement: "I've run for an hour"}
  - {val: 10, achievement: "I've run for ten hours"}
  - {val: 24, achievement: "I've run a whole day"}
  distance:
  - {val: 1000, achievement: "I've run for one kilometre"}
  - {val: 10000, achievement: "I've run for ten kilometres"}
  - {val: 42195, achievement: "I've just completed an actual marathon."}
  - {val: 100000, achievement: "I've run for one hundred kilometres"}
  bumper_hits:
  - {val: 1, achievement: "I've been bumped once"}
  - {val: 10, achievement: "I've been bumped ten times. Watch out people"}
  - {val: 50, achievement: "I've been bumped FIFTY times. Are you doing this on purpose"}
```
There are two types of value for `<achievemment_type>`. If it is either `run_duration` or `distance` then the `val` argument is interpreted as hours or metres respectively. Otherwise the `<achievemment_type>` is used to retrieve a field from the Python class `Episode` in `waypoint_patroller.log_util` and then the `val` argument is either compared directly to this (in the case that it's a bare value) or the length of the indicated list, in the case that the field is a list.

For the marathon, you are free to change the text of the achievement announcement, but please don't change the target values. We should all be attempting the same things.

