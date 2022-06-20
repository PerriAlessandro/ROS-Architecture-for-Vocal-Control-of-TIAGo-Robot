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
	##MODE 1 - PLAY_MOTION
	if 'hello' in req.word:
		mode = 1
		kind = 1
	elif 'unfold' in req.word:
		mode = 1
		kind = 2
	elif 'maximum' in req.word:
		mode = 1
		kind = 3
	elif 'floor' in req.word:
		mode = 1
		kind = 4
	elif 'shake' in req.word:
		mode = 1
		kind = 5
	elif 'offer' in req.word:
		mode = 1
		kind = 6
	elif 'surroundings' in req.word:
		mode = 1
		kind = 7
	elif 'tour' in req.word:
		mode = 1
		kind = 8
	elif 'close' in req.word:
		mode = 1
		kind = 9
	elif 'half' in req.word:
		mode = 1
		kind = 10
	elif 'gym' in req.word:
		mode = 1
		kind = 11
	elif 'home' in req.word:
		mode = 1
		kind = 12

	##MODE 2
	#speed settings 
	elif 'accelerate' in req.word:
		mode = 2
		kind = 6
	elif 'decelerate' in req.word:
		mode = 2
		kind = 7
	#reset velocity to 1.0
	elif 'reset' in req.word:
		mode = 2
		kind = 8
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

	#MODE 3 - ARM MOTION
	elif 'arm' in req.word:
		mode = 3
		kind =1



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
