#! /usr/bin/env python3

import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from speech_rec.msg import MoveSpeechAction, MoveSpeechGoal , MoveSpeechActionFeedback

stop_motion=False
vel_adder=0.20
vel=0.5

can_go_left = True
can_go_right = True
can_go_straight = True
prev_can_go_left=can_go_left
prev_can_go_right=can_go_right
prev_can_go_straight=can_go_straight

goal = MoveSpeechGoal()




def callback_feedback(feedback):

    global stop_motion

    if stop_motion:
        print("STOP")
        client.cancel_all_goals()
        stop_motion=False


def callback_laser(laser):
    global can_go_left
    global can_go_right
    global can_go_straight
    global prev_can_go_left
    global prev_can_go_right
    global prev_can_go_straight
    global stop_motion
    #retrieving the minimum frontal and lateral distances, if the distance is greater or equal to 1 meter, then that distance will be set to 1
    right = min(min(laser.ranges[40:122]), 1)
    front = min(min(laser.ranges[273:494]), 1)
    left = min(min(laser.ranges[545:622]), 1)

    #if the value is not '1', it means that it is lower and therefore the robot should not be able to go towards that direction
    if right != 1:
        can_go_right =False
        if(can_go_right != prev_can_go_right):
            print("MOTION RIGHT STOPPED")
            client.cancel_all_goals()
        

    else:
        can_go_right =True

    if left != 1:
        can_go_left =False
        if(can_go_left != prev_can_go_left):
            print("MOTION LEFT STOPPED")
            client.cancel_all_goals()
        
    else:
        can_go_left =True

    if front != 1:
        can_go_straight =False
        if(can_go_straight != prev_can_go_straight):
            print("MOTION STRAIGHT STOPPED")
            client.cancel_all_goals()
        
    else:
        can_go_straight =True

    prev_can_go_right=can_go_right
    prev_can_go_left=can_go_left
    prev_can_go_straight=can_go_straight


def motion_callback(msg):
    global can_go_left
    global can_go_right
    global can_go_straight
    global vel
    global stop_motion
    global goal


    client.wait_for_server()

    if msg.data == 1 and can_go_straight:
        print("GO FORWARD")
        goal.velocity = vel
        goal.turn = 0
        goal.time = 10
        client.send_goal(goal,None,None,callback_feedback)

    if msg.data == 10 :
        print("GO BACKWARDS")
        goal.velocity = -vel
        goal.turn = 0
        goal.time = 10
        client.send_goal(goal,None,None,callback_feedback)

    if msg.data == 2 and can_go_left:
        print("TURN LEFT")
        goal.velocity = 0
        goal.turn = 0.785
        goal.time = 2
        client.send_goal(goal,None,None,callback_feedback)
    if msg.data == 3 and can_go_right:
        print("TURN RIGHT")
        goal.velocity = 0
        goal.turn = -0.785
        goal.time = 2
        client.send_goal(goal,None,None,callback_feedback)

    if msg.data == 4 and can_go_right and can_go_straight:
        print("GO STRAIGHT-RIGHT")
        goal.velocity = vel
        goal.turn = -0.785
        goal.time = 2
        client.send_goal(goal,None,None,callback_feedback)

    if msg.data == 5 and can_go_left and can_go_straight:
        print("GO STRAIGHT-LEFT")
        goal.velocity = vel
        goal.turn = +0.785
        goal.time = 2
        client.send_goal(goal,None,None,callback_feedback)
        
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
 

    #if client.send_goal_and_wait(goal, rospy.Duration(50.0), rospy.Duration(50.0)) == 3:
    #    rospy.loginfo('Call to action server succeeded')
    #else:
    #    rospy.logerr('Call to action server failed')


if __name__ == '__main__':
    rospy.init_node('move_speech_client')

    client = actionlib.SimpleActionClient('move_spc', MoveSpeechAction)

    sub1 = rospy.Subscriber('/moving', Int32, motion_callback)

    sub2 = rospy.Subscriber('/scan_raw', LaserScan, callback_laser)

    rospy.spin()

    
    