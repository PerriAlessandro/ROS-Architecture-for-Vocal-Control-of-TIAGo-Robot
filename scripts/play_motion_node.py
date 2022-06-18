#!/usr/bin/env python3
#!/usr/bin/env python2
 
import rospy
import sys
import time
from std_msgs.msg import Int32
from speech_rec.srv import Word , WordRequest
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal



def wait_for_valid_time(timeout):

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

def play_callback(msg):
	
	if msg.data == 1:
		sendgoal('wave')
	if msg.data == 2:
		sendgoal('unfold_arm')
	if msg.data == 3:
		sendgoal('reach_max')
	if msg.data == 4:
		sendgoal('reach_floor')
	if msg.data == 5:
		sendgoal('shake_hands')
	if msg.data == 6:
		sendgoal('offer')
	if msg.data == 7:
		sendgoal('inspect_surroundings')
	if msg.data == 8:
		sendgoal('head_tour')
	if msg.data == 9:
		sendgoal('close')
	if msg.data == 10:
		sendgoal('close_half')
	if msg.data == 11:
		sendgoal('do_weights')
	if msg.data == 12:
		sendgoal('home')
def main():
	rospy.init_node("play_motion_node")

	rate = rospy.Rate(5)

	sub = rospy.Subscriber('/playmotion', Int32, play_callback)

	while not rospy.is_shutdown():
		rate.sleep()	

if __name__ == '__main__':
	main()




	

