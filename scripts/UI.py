#!/usr/bin/env python3

############## UI ################

#Libraries
import speech_recognition as sr
import rospy
import sys
import time
import os
from std_msgs.msg import Int32
from speech_rec.srv import Word , WordRequest
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def word_client(word):

	'''
	Function to create the client for the custom ROS-Action "Word" available 
	in the "action" folder, in order to send the request to the server and get 
	its response.

	Arguments:
		* word: Word linstened by the microphone sent to the server to be elaborated.
	Returns:
		* result: the server response.

	'''

	# wanting for the service to be online 
	rospy.wait_for_service('text')

	try:
		# creation of the client for the custom action
		word_srv = rospy.ServiceProxy('text',Word)
		# sending the request to the server 
		resp = WordRequest(word)
		# getting the reponse from the server 
		result = word_srv(resp)
		# returning the server's response
		return result

	except rospy.ServiceException as e:
		# printing the proper exception if any
		print('Service Failed: %s'%e)

def take_command():

    '''
    Function to get the vocal input from the microphone, thanks to the Uberi speech recognition 
    library, the Google-Recognition service is used to process the input from vocal to a string.

    Arguments:
       * None
    Returns:
       * command: the whole vocal input as a string 
    '''

    try:
        # Using the Computer microphone a input source 
        with sr.Microphone() as source:
            print('listening...')
            # Adjusting the external ambient noise 
            listener.adjust_for_ambient_noise(source)
            # Listening from the source 
            audio = listener.listen(source)
            # Using Google API to recognize the vocal input 
            command = listener.recognize_google(audio, language="en-EN")
            print("Ok! I'm processing the message!")
            # retrieving the in input in lower case string 
            command = command.lower()
            print("You said: \n", command)
            # returning the vocal command only if the key-word is in the string line
            # vocal input 
            if 'alexa' in command:
                # erasing the key-word from the string 
                command = command.replace('alexa', '')
                return command
    except Exception as e:
    	# printing the exceptions if any
        print(e)
    return "I didn't get it"

def run_tiago():

	"""
	Function to send a message to activate the proper node, according to the server
	response.

	Arguments:
		* None
	Returns:
		* None
	"""

	# retrieving the vocal cammand
	command = take_command()
	print(command)

	# getting the server response
	x = word_client(command)
	print(x.mode)
	print(x.kind)

	# Sending a message to the proper node
	if x.mode == 1:
		msg = x.kind
		pub1.publish(msg)
	elif x.mode == 2:
		msg = x.kind
		pub2.publish(msg)
	elif x.mode == 3:
		msg = x.kind
		pub3.publish(msg)
	else:
		print("Wrong Modality")


if __name__ == '__main__':

	os.system('cls||clear')

	# Initializing the Node
	rospy.init_node("UI")

	# Initializing the working-rate the node
	rate = rospy.Rate(5)

	# Defining a listener object from the speech recognition library
	listener = sr.Recognizer()

	# Initializing the publishers to communicate with the other nodes
	pub1 = rospy.Publisher('/playmotion', Int32, queue_size=10)
	pub2 = rospy.Publisher('/moving', Int32, queue_size=10)
	pub3 = rospy.Publisher('/arm_moving', Int32, queue_size=10)
	
	while not rospy.is_shutdown():
		
		print ("inside if")
		run_tiago()
		rate.sleep()
	

	
