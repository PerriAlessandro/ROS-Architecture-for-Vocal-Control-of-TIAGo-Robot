#! /usr/bin/env python3

import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from speech_rec.msg import *

class MoveSpeechServer:

  _status_feedback = MoveSpeechActionFeedback() # feedback sullo stato del goal
  _feedback = MoveSpeechFeedback() # feedback sulla position e rotation

  _status_result = MoveSpeechActionResult() 
  _result = MoveSpeechResult()

  def __init__(self,name):

    self._action_name = name
    self.server = actionlib.SimpleActionServer(self._action_name, MoveSpeechAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    rate = 10
    r = rospy.Rate(rate)
    
    self._feedback.time = 0

    pub_msg = Twist()
    pub_msg.linear.x = goal.velocity
    pub_msg.linear.y = 0
    pub_msg.linear.z = 0

    pub_msg.angular.x = 0
    pub_msg.angular.y = 0
    pub_msg.angular.z = goal.turn

    print("Duration of the action: "+str(goal.time * rate))

    for i in range(0, goal.time * rate):

      #print(goal.time * rate)
      #print('time is passing..  ',i)
      #print(goal.time * rate)

      if self.server.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self.server.set_preempted()
        success = False
        break

      #self._feedback.time = i
     
      #self.server.publish_feedback(self._feedback)
      pub.publish(pub_msg)
      r.sleep()

    success = True
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('moving_server')
  pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
  server = MoveSpeechServer('move_spc')
  