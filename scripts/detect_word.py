#!/usr/bin/env python3
#!/usr/bin/env python2

from speech_rec.srv import Word , WordResponse
import speech_recognition as sr
import sys
import time
# ROS imports
import rospy
 

def server_callback(req):
	if (req == 'wave'):
		resp = 5
		return WordResponse(resp)
	else:
		resp = 0
		return WordResponse(resp)

def word_server():
	rospy.init_node('detect_word')
	s = rospy.Service('text', Word, server_callback)
	rospy.spin()

if __name__ == "__main__":
	word_server()
