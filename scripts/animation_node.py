#!/usr/bin/env python3

#Libraries
import rospy
import time
from std_msgs.msg import Int32
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal



def wait_for_valid_time(timeout):
    '''
    Function to wait for a valid time, exits if it waits for more than timeout seconds

   Arguments: 
     * timeout: timeout in seconds 
    
    Returns:
     * None
    '''

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
    '''
    Function to convert the status code into a string
   	Arguments: 
     * status_code: status of the action
    
    Returns:
     * String corresponding to the status code
    '''
    return GoalStatus.to_string(status_code)

def start_action(text):
    '''
    Function to create a PlayMotionAction client, generate the PlayMotionGoal goal by 
	passing as 'motion_name' attribute the string 'text', which is passed as argument and trivially contains the
	name of the motion. 
   	Arguments: 
     * text: the name of the motion to be executed
    
    Returns:
     * None 
    '''

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
	'''
	Callback function of subscription to /playmotion topic. Each integer in msg.data corresponds
	to a certain motion that has to be executed through an action.

    Arguments: 
     * msg: the integer corresponding to a certain motion
    
    Returns:
     * None
	'''
	
	if msg.data == 1:
		start_action('wave')
	if msg.data == 2:
		start_action('unfold_arm')
	if msg.data == 3:
		start_action('reach_max')
	if msg.data == 4:
		start_action('reach_floor')
	if msg.data == 5:
		start_action('shake_hands')
	if msg.data == 6:
		start_action('offer')
	if msg.data == 7:
		start_action('inspect_surroundings')
	if msg.data == 8:
		start_action('head_tour')
	if msg.data == 9:
		start_action('close')
	if msg.data == 10:
		start_action('close_half')
	if msg.data == 11:
		start_action('do_weights')
	if msg.data == 12:
		start_action('home')

		
def main():
	rospy.init_node("animation_node")

	rate = rospy.Rate(5)
	#subscription to /playmotion topic which sends the related integer anytime the user gives a vocal command
	sub = rospy.Subscriber('/playmotion', Int32, play_callback)

	while not rospy.is_shutdown():
		rate.sleep()	

if __name__ == '__main__':
	main()




	

