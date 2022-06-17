#!/usr/bin/env python3
#!/usr/bin/env python2

from speech_rec.srv import Word , WordResponse
import speech_recognition as sr
import sys
import time
# ROS imports
import rospy
 

def server_callback(req):

	print('detected: ', req.word)
	if 'hello' in req.word:
		mode = 1
		kind = 1
	
	elif 'go' in req.word:
		mode = 2
		kind = 1
	elif 'left' in req.word:
		mode=2
		if 'straight' in req.word:
			kind = 5
		else:
			kind = 2
	elif 'right' in req.word:
		mode=2
		if 'straight' in req.word:
			kind = 4
		else:
			kind = 3
	elif 'stop' in req.word:
		mode = 2
		kind = -1
		
	else:
		mode = -1
		kind = -1

	return WordResponse(mode,kind)

def word_server():
	rospy.init_node('detect_word')
	s = rospy.Service('text', Word, server_callback)
	rospy.spin()

if __name__ == "__main__":
	word_server()
