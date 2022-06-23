#!/usr/bin/env python3
#!/usr/bin/env python2

# Libraries
from speech_rec.srv import Word , WordResponse
import vocal
import speech_recognition as sr
import sys
import time
import rospy
 
def server_callback(req):
	'''
	This callback function retrieves the request (req) from the client UI.py.
	The response will be set as a couple of numbers indicating the action the robot will execute.
	The two numbers identify:
		* mode (int32): The type of action te robot will execute.
		* kind (int32): The specific goal the robot will complete the specific movement.

	Arguments: 
		* req (string): request from the UI.py client

	'''

	print('detected: ', req.word)

	# The function word of the vocal.py script will call another function of the script
	# corresponding to the given command which will set the mode and kind variables to 
	# the correct int values corresponding to the required action.
	mode,kind=vocal.word(req.word)

	return WordResponse(mode,kind)

def word_server():
	'''
	This function will init the ros node and will define the service related to the server_callback function.
	'''

	rospy.init_node('detect_word')
	s = rospy.Service('text', Word, server_callback)
	rospy.spin()

if __name__ == "__main__":
	word_server()
