
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


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create an arm controller action client to move the TIAGo's arm
arm_control_client_Ptr ArmClient;
// set of joint positions corresponding to a certain motion
double joint_pos_1[1][7] = {0.2,0.0,-1.5,1.94,-1.57,-0.5,0.0};
// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  cout<<"START WAITING"<<endl;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }
  cout<<"STOP WAITING"<<endl;

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal,double jpos [][7])
{
    cout<<"AAAAAAAAAAAAAAAAAAAA"<<endl;
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

 int n_waypoints=sizeof(jpos)/sizeof(jpos[0])+1;
 int n_joints=sizeof(jpos[0])/sizeof(jpos[0][0]);
cout<<"wp: "<<n_waypoints<<endl;
cout<<"joints: "<<n_joints<<endl;
  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(n_waypoints);

double duration=2.0;
for(int j=0;j<n_waypoints;j++)
{
    goal.trajectory.points[j].positions.resize(n_joints);
    goal.trajectory.points[j].velocities.resize(n_joints);
    for(int i=0;i<n_joints;i++)
    {
       goal.trajectory.points[j].positions[i] = jpos[j][i];
       cout<<"joint"<<i<<": "<<goal.trajectory.points[j].positions[i]<<endl;
if(j==(n_waypoints-1))  goal.trajectory.points[j].velocities[i+1] = 0.0; else  goal.trajectory.points[j].velocities[i+1] = 1.0; 
    }

//       for (int k = 0; k < 7; ++k)
//   {
//     if(j==(n_waypoints-1))  goal.trajectory.points[j].velocities[k] = 0.0; else  goal.trajectory.points[j].velocities[k] = 1.0;

//   }
cout<<endl;
  goal.trajectory.points[j].time_from_start = ros::Duration(duration);
  duration=duration+2.0;

}

}



//motion callBack 
void motionCallback(const std_msgs::Int32::ConstPtr& msg)
{   
      cout<<"msg data: "<<msg->data<<endl;

    control_msgs::FollowJointTrajectoryGoal arm_goal;
    if (msg->data == 1){
         // Generates the goal for the TIAGo's arm
         cout<<"into the if"<<endl;
        waypoints_arm_goal(arm_goal,joint_pos_1);

    }


  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);
    cout<<"goal sent!"<<endl;
  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "arm_traj_control");

  ROS_INFO("Starting arm_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  
  createArmClient(ArmClient);

  // Subscription to /arm_moving topic (needed to receive the type of motion to be performed from UI node)
  ros::Subscriber sub = nh.subscribe("/arm_moving",1000, motionCallback);


  ros:: spin();
} 
