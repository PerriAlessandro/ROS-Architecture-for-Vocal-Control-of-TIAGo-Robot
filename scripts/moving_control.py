#! /usr/bin/env python3

import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from speech_rec.msg import MoveSpeechAction, MoveSpeechGoal , MoveSpeechActionFeedback

stop_motion=False
vel_adder=0.20
vel=0.5

def callback_feedback(feedback):
    if stop_motion:
        print("AAAAAAAASTOP")
        client.cancel_all_goals()
        stop_motion=False


def motion_callback(msg):
    global vel
    to_send=False
    client.wait_for_server()
    goal = MoveSpeechGoal()

    if msg.data == 1:
        print("GO FORWARD")
        goal.velocity = vel
        goal.turn = 0
        goal.time = 10
        to_send=True
    if msg.data == 2:
        print("TURN LEFT")
        goal.velocity = 0
        goal.turn = 0.785
        goal.time = 2
        to_send=True
    if msg.data == 3:
        print("TURN RIGHT")
        goal.velocity = 0
        goal.turn = -0.785
        goal.time = 2
        to_send=True
    if msg.data == 4:
        print("GO STRAIGHT-RIGHT")
        goal.velocity = vel
        goal.turn = -0.785
        goal.time = 2
        to_send=True
    if msg.data == 5:
        print("GO STRAIGHT-LEFT")
        goal.velocity = vel
        goal.turn = +0.785
        goal.time = 2
        to_send=True
    if msg.data == 6:
        vel=vel+vel_adder
        print("INCREASE VELOCITY: "+str(vel))
    if msg.data == 7:
        if (vel-vel_adder)>0:
            vel=vel-vel_adder
            print("DECREASE VELOCITY: "+str(vel))
        else:
            print("Cannot decrease more!")
    if msg.data == 8:
        vel=1.0
        print("RESET VELOCITY: "+str(vel))

    if msg.data == -1:
        stop_motion=True
        print("STOP MOTION")

    if to_send:
        client.send_goal(goal,None,None,callback_feedback)
 

    #if client.send_goal_and_wait(goal, rospy.Duration(50.0), rospy.Duration(50.0)) == 3:
    #    rospy.loginfo('Call to action server succeeded')
    #else:
    #    rospy.logerr('Call to action server failed')


if __name__ == '__main__':
    rospy.init_node('move_speech_client')

    client = actionlib.SimpleActionClient('move_spc', MoveSpeechAction)

    sub = rospy.Subscriber('/moving', Int32, motion_callback)

    rospy.spin()

    
    