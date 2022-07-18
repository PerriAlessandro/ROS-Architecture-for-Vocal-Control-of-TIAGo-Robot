
// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <std_msgs/Int32.h>
using namespace std;

// Our Action interface type for moving TIAGo's arm, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

// Create an arm controller action client to move the TIAGo's arm
arm_control_client_Ptr ArmClient_left;
arm_control_client_Ptr ArmClient_right;

//constant value representing the increment of a certain joint
const double DELTA_SHOULDER=0.8;
const double DELTA_ELBOW=0.8;
//array containing the current joint position
double left_current_joint_pos[1][7] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double right_current_joint_pos[1][7] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//elementary steps to be added to the current joint configuration
double delta_q_shoulder[1][7] = {0.0,DELTA_SHOULDER,0.0,0.0,0.0,0.0,0.0};
double delta_q_elbow[1][7] = {0.0,0.0,0.0,DELTA_ELBOW,0.0,0.0,0.0};



void array_sum(double j_add[][7],int rows, int cols,bool add, bool arm){
    /*
    Function to make the scalar sum between "j_add" array and the one representing the current joint configuration

    Arguments: 
     * j_add: rowsxcols array which has to be scalarly added to the current joint configuration
     * rows: number of rows
     * cols: number of columns
     * add:  if true, returns the sum. If false, returns the difference
     * arm: false for left arm, true for right arm
    
    Returns:
     * None
    */
    ROS_INFO("Current joints position: ");
    for (int i=0;i<rows;i++)
    {
       for (int j=0;j<cols;j++)
        {
        if(add){
          if (arm){
          	right_current_joint_pos[i][j]=right_current_joint_pos[i][j]+j_add[i][j];
		ROS_INFO("%f",right_current_joint_pos[i][j]);}
	  else {
	        left_current_joint_pos[i][j]=left_current_joint_pos[i][j]+j_add[i][j];
	        ROS_INFO("%f",left_current_joint_pos[i][j]);}
          }
        else{

          if (arm){
          	right_current_joint_pos[i][j]=right_current_joint_pos[i][j]-j_add[i][j];
	        ROS_INFO("%f",right_current_joint_pos[i][j]);}
	  else {
	        left_current_joint_pos[i][j]=left_current_joint_pos[i][j]-j_add[i][j];
                ROS_INFO("%f",left_current_joint_pos[i][j]);}

	}

        
        } 
    }
}



void createArmClient(arm_control_client_Ptr& actionClient, const std::string arm_controller_name){
  /*
  Function to create a ROS action client to move TIAGos arm

  Arguments: 
    * actionClient: a shared pointer to arm_control_client, which uses messages of type FollowJointTrajectoryAction
  
  Returns:
    * None
  */
  ROS_DEBUG("Creating action client to arm controller ...");
  std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";

  actionClient.reset( new arm_control_client(action_client_name) );

  int iterations = 0, max_iterations = 3;

  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )

    ROS_FATAL("Error in createArmClient: arm controller action server not available");
    // throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal,double jpos [][7], const std::string arm_controller_name){
  /*
  Function to generate a trajectory with a given number of waypoints to move TIAGos arm 

  Arguments: 
   * goal: reference to FollowJointTrajectoryGoal goal message
   * jpos: bidimensional array containing the joint position configuration (columns) for each waypoint (rows)

  Returns:
    * None
  */
  
 int n_waypoints=sizeof(jpos)/sizeof(jpos[0])+1;
 int n_joints=sizeof(jpos[0])/sizeof(jpos[0][0]);
 ROS_INFO("wp: %d",n_waypoints);
 ROS_INFO("joints: %d", n_joints);

  // The joint names, which apply to all waypoints
  for(int i=0; i < n_joints ; i++){
    goal.trajectory.joint_names.push_back("arm_"+arm_controller_name +"_"+to_string(i+1)+"_joint");
  }




// resizing so that each row corresponds to a waypoint in this goal trajectory
goal.trajectory.points.resize(n_waypoints);

// duration for each waypoint
double duration=2.0;
for(int j=0;j<n_waypoints;j++)
{ 
    //resizing for positions and velocities
    goal.trajectory.points[j].positions.resize(n_joints);
    goal.trajectory.points[j].velocities.resize(n_joints);
    //the goal position configuration is set for each joint
    for(int i=0;i<n_joints;i++)
    {
      goal.trajectory.points[j].positions[i] = jpos[j][i];
      ROS_INFO("joint %d: %f",i, goal.trajectory.points[j].positions[i]);
      for (int k = 0; k < 7; ++k)
      {
        if(j==(n_waypoints-1))
          //we want that the arm reaches the last waypoint with a 0 velocity
          goal.trajectory.points[j].velocities[k] = 0.0;
        else
          goal.trajectory.points[j].velocities[k] = 0.8;
      }

    }
  //each waypoint should be reached within 2 seconds
  goal.trajectory.points[j].time_from_start = ros::Duration(duration);
  duration=duration+2.0;

}

}




void motionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  /*
  Callback function of subscription to /arm_moving topic which 
  is needed to receive the type of motion to be performed from UI node.

  Arguments: 
    * msg: value of the callback
  
  Returns:
    * None
  */

  ROS_INFO("msg data: %d", msg->data);
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  // Generates the goal for the TIAGo's arm
  if (msg->data == 1){ // increase SHOULDER position (LEFT)
    //check if the joint is already at its limits
    if(left_current_joint_pos[0][1]<DELTA_SHOULDER){
        //add the increment to the current configuration 
        array_sum(delta_q_shoulder,1,7,false,false);
        waypoints_arm_goal(arm_goal,left_current_joint_pos, "left");
    }
        else
          ROS_WARN("+ SHOULDER NOT ALLOWED (LEFT)");
    }
    if (msg->data == 2){ // decrease SHOULDER position (LEFT)
        //check if the joint is already at its limits
        if(left_current_joint_pos[0][1]>-DELTA_SHOULDER){
        //add the increment to the current configuration 
          array_sum(delta_q_shoulder,1,7,true,false);
          waypoints_arm_goal(arm_goal,left_current_joint_pos,"left");
        }
      else
          ROS_WARN("- SHOULDER NOT ALLOWED (LEFT)");
    }
    if (msg->data == 3){ // + ELBOW (LEFT)
        //check if the joint is already at its limits
        if(left_current_joint_pos[0][3]<DELTA_ELBOW){
        //add the increment to the current configuration 
          array_sum(delta_q_elbow,1,7,true,false);
          waypoints_arm_goal(arm_goal,left_current_joint_pos,"left");
        }
        else
          ROS_WARN("+ ELBOW NOT ALLOWED (LEFT)");
    }

    if (msg->data == 4){ // -ELBOW (LEFT)
        //check if the joint is already at its limits
        if(left_current_joint_pos[0][3]> - DELTA_ELBOW){
        //add the increment to the current configuration 
          array_sum(delta_q_elbow,1,7,false,false);
          waypoints_arm_goal(arm_goal,left_current_joint_pos,"left");
        }
        else
          ROS_WARN("- ELBOW NOT ALLOWED (LEFT)");
    }

  if (msg->data == 5){ // increase SHOULDER position (RIGHT)
    //check if the joint is already at its limits
    if(right_current_joint_pos[0][1]<DELTA_SHOULDER){
        //add the increment to the current configuration 
        array_sum(delta_q_shoulder,1,7,false,true);
        waypoints_arm_goal(arm_goal,right_current_joint_pos,"right");
    }
        else
          ROS_WARN("+ SHOULDER NOT ALLOWED (RIGHT)");
    }

    if (msg->data == 6){ // decrease SHOULDER position (RIGHT)
        //check if the joint is already at its limits
        if(right_current_joint_pos[0][1]>-DELTA_SHOULDER){
        //add the increment to the current configuration 
          array_sum(delta_q_shoulder,1,7,true,true);
          waypoints_arm_goal(arm_goal,right_current_joint_pos,"right");
        }
      else
          ROS_WARN("- SHOULDER NOT ALLOWED (RIGHT)");
    }
    if (msg->data == 7){ // + ELBOW (RIGHT)
        //check if the joint is already at its limits
        if(right_current_joint_pos[0][3]<DELTA_ELBOW){
        //add the increment to the current configuration 
          array_sum(delta_q_elbow,1,7,true,true);
          waypoints_arm_goal(arm_goal,right_current_joint_pos,"right");
        }
        else
          ROS_WARN("+ ELBOW NOT ALLOWED (RIGHT)");
    }

    if (msg->data == 8){ // -ELBOW (RIGHT)
        //check if the joint is already at its limits
        if(right_current_joint_pos[0][3]> - DELTA_ELBOW){
        //add the increment to the current configuration 
          array_sum(delta_q_elbow,1,7,false,true);
          waypoints_arm_goal(arm_goal,right_current_joint_pos,"right");
        }
        else
          ROS_WARN("- ELBOW NOT ALLOWED (RIGHT)");
    }

  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  // Sends the goal
if (msg->data > 0 and msg->data < 5){
  ArmClient_left->sendGoal(arm_goal);
  ROS_DEBUG("LEFT: arm_goal goal sent!");
  // Wait for trajectory execution
  while(!(ArmClient_left->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }}
else {
  ArmClient_right->sendGoal(arm_goal);
  ROS_DEBUG("RIGHT: arm_goal goal sent!");
  // Wait for trajectory execution
  while(!(ArmClient_right->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
}
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "arm_traj_control");

  ROS_INFO("Initializing arm_traj_control node ...");
  
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Creates the client for TIAGO's arm motion
  ROS_DEBUG("Initializing ArmClient client ...");
  createArmClient(ArmClient_left,"arm_left_controller");
  createArmClient(ArmClient_right,"arm_right_controller");

  ROS_DEBUG("Subscribing to /arm_moving topic through the motionCallback function ...");
  // Subscription to /arm_moving topic (needed to receive the type of motion to be performed from UI node)
  ros::Subscriber sub = nh.subscribe("/arm_moving",1000, motionCallback);


  ros:: spin();
} 
