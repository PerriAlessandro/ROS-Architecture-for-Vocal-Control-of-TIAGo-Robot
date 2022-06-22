#!/usr/bin/env python3
#!/usr/bin/env python2

from speech_rec.srv import Word , WordResponse
import vocal
import speech_recognition as sr
import sys
import time
# ROS imports
import rospy
 

def server_callback(req):

	print('detected: ', req.word)
	mode,kind=vocal.word(req.word)

	return WordResponse(mode,kind)

def word_server():
	rospy.init_node('detect_word')
	s = rospy.Service('text', Word, server_callback)
	rospy.spin()

if __name__ == "__main__":
	word_server()
