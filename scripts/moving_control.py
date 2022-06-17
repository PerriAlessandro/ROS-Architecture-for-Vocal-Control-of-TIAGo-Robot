#! /usr/bin/env python3

import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from speech_rec.msg import MoveSpeechAction, MoveSpeechGoal , MoveSpeechActionFeedback

def motion_callback(msg):
    
    client.wait_for_server()
    goal = MoveSpeechGoal()
    # Fill in the goal here
    print(" callback kind")
    if msg.data == 1:
        print("message retrieved")
        goal.velocity = 0.5
        goal.turn = 0
        goal.time = 2


    if msg.data == 2:
        print("STOPPED")

    if msg.data == 3:
        print("message retrieved")
        goal.velocity = 0
        goal.turn = 0.785
        goal.time = 2

    #if msg.data == 3:
    #    goal.turn_distance = 2
    #    goal.forward_distance = 0
    print("sending the goal")
    client.send_goal_and_wait(goal, rospy.Duration(50.0))
    print("goal recived")

    #if client.send_goal_and_wait(goal, rospy.Duration(50.0), rospy.Duration(50.0)) == 3:
    #    rospy.loginfo('Call to action server succeeded')
    #else:
    #    rospy.logerr('Call to action server failed')


if __name__ == '__main__':
    rospy.init_node('move_speech_client')

    client = actionlib.SimpleActionClient('move_spc', MoveSpeechAction)

    sub = rospy.Subscriber('/moving', Int32, motion_callback)

    rospy.spin()

    
    