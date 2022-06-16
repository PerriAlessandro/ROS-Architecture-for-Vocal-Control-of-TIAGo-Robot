#!/usr/bin/env python3
#!/usr/bin/env python2
 

import speech_recognition as sr
import rospy
from speech_rec.srv import Word , WordRequest

listener = sr.Recognizer()
rospy.init_node("UI")

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
                print(command)
                return command
    except Exception as e:
        print(e)
    return "I didn't get it"

def run_tiago():

	command = take_command()
	print(command)
	
	print(word_client(command))
	
while True:
    run_tiago()


