#!/usr/bin/env python3
#!/usr/bin/env python2
import speech_recognition as sr
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


def wait_for_valid_time(timeout):
    """Wait for a valid time (non-zero), this is important
    when using a simulated clock"""
    # Loop until:
    # * ros master shutdowns
    # * control+C is pressed (handled in is_shutdown())
    # * timeout is achieved
    # * time is valid
    start_time = time.time()
    while not rospy.is_shutdown():
        if not rospy.Time.now().is_zero():
            return
        if time.time() - start_time > timeout:
            rospy.logerr("Timed-out waiting for valid time.")
            exit(0)
        time.sleep(0.1)
    # If control+C is pressed the loop breaks, we can exit
    exit(0)

def get_status_string(status_code):
    return GoalStatus.to_string(status_code)

def sendgoal(text):

    rospy.loginfo("Starting run_motion_python application...")
    wait_for_valid_time(10.0)

    client = SimpleActionClient('/play_motion', PlayMotionAction)

    rospy.loginfo("Waiting for Action Server...")
    client.wait_for_server()

    goal = PlayMotionGoal()

    goal.motion_name = text
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + text)
    client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    action_ok = client.wait_for_result(rospy.Duration(30.0))

    state = client.get_state()

    if action_ok:
        rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(get_status_string(state)))

    

rospy.init_node("speechrec")

recognizer_instance = sr.Recognizer() # Crea una istanza del recognizer
text=''
while text != 'exit':
    with sr.Microphone() as source:
        recognizer_instance.adjust_for_ambient_noise(source)
        print("I'm listening.. just talk!")
        audio = recognizer_instance.listen(source)
        print("Ok! I'm processing the message!")
    try:
        text = recognizer_instance.recognize_google(audio, language="en-EN")
        print("You said: \n", text)
        if (text == 'wave'):
            sendgoal(text)
    except Exception as e:
        print(e)




print("Hope to (not) see you again!")
