#!/usr/bin/env python3
#!/usr/bin/env python2
 

import speech_recognition as sr
import rospy
import sys
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from speech_rec.srv import Word , WordRequest
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def word_client(word):
	
	rospy.wait_for_service('text')

	try:
		word_srv = rospy.ServiceProxy('text',Word)
		resp = WordRequest(word)
		result = word_srv(resp)
		return result

	except rospy.ServiceException as e:
		print('Service Failed: %s'%e)

def take_command():
    try:
        with sr.Microphone() as source:
            print('listening...')
            listener.adjust_for_ambient_noise(source)
            audio = listener.listen(source)
            command = listener.recognize_google(audio, language="en-EN")
            print("Ok! I'm processing the message!")
            command = command.lower()
            print("You said: \n", command)
            if 'alexa' in command:
                command = command.replace('alexa', '')
                return command
    except Exception as e:
        print(e)
    return "I didn't get it"

def run_tiago():

	command = take_command()
	print(command)

	x = word_client(command)
	print(x.mode)
	print(x.kind)

	if x.mode == 1:
		msg = x.kind
		pub1.publish(msg)
	elif x.mode == 2:
		msg = x.kind
		pub2.publish(msg)
	#elif x.mode == 3:
		#msg = x.kind
		#pub3.publish(msg)
	else:
		print("Wrong Modality")

	
	
	#if word_client(command) == 1:
	#	sendgoal('wave')

	#elif word_client(command) == 2:
	#	pub_msg = Twist()

	#	pub_msg.linear.x = 2
	#	pub_msg.linear.y = 0
	#	pub_msg.linear.z = 0

	#	pub_msg.angular.x = 0
	#	pub_msg.angular.y = 0
	#	pub_msg.angular.z = 0


	#	pub.publish(pub_msg)

	#elif word_client(command) == 3:
	#	pub_msg = Twist()

	#	pub_msg.linear.x = 0
	#	pub_msg.linear.y = 0
	#	pub_msg.linear.z = 0

	#	pub_msg.angular.x = 0
	#	pub_msg.angular.y = 0
	#	pub_msg.angular.z = 0
		
	#	pub.publish(pub_msg)

	#	rospy.spin()

if __name__ == '__main__':


	rospy.init_node("UI")
	pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

	listener = sr.Recognizer()

	pub1 = rospy.Publisher('/playmotion', Int32, queue_size=10)
	pub2 = rospy.Publisher('/moving', Int32, queue_size=10)
	


	rate = rospy.Rate(5)

	while not rospy.is_shutdown():

	    run_tiago()

	    rate.sleep()
