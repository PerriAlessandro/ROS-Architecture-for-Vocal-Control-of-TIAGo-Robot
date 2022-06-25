__SOFAR Assignment - TIAGo Vocal Controller__ 
================================
__Aim of the project__
----------------------
The goal of this project is to build a simple software architecture to perform some basic control on
TIAGo robot through vocal commands. To be more precise, __Google Speech Recognition__ APIs have been used to recognize the vocal commands of the user. The keywords detected by TIAGo are many and can be subdivided into three main categories:
- `Motion of the base`: the robot is able to perform some basics movements of the base, such as going forward and backward, turn left and right. Moreover, a simple obstacle avoidance algorithm has been implemented.
- `Motion of the arms`: straight-forward additive motions such as increasing the position of the joints corresponding to the shoulder and the elbow with respect to their zero position.
- `Pre-defined motions`: based on [play_motion](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion) package, TIAGo is now able to play predefined upper-body motions through vocal commands.
These commands sections correspond to the three different **ROS Actions** deployed by the software to animate the robot exploiting different topics and comunication paths.

__Description of the Software architecture__
----------------------
To create the software achitecture for commanding the TIAGo robot we developed **ROS Pakage** called *speech_rec*. Inside the folder we developed multiple **ROS Nodes** and scripts to propose the most modular and schematic solution to the given problem.
The following list summerises the role of each node inside the achitecture:

* `UI.py` & `detected_word.py`: The User Interface script will recognize the vocal commands of the user thanks to the [*voice recognizer*](https://github.com/Uberi/speech_recognition) which exploits the __Google Speech Recognition__ APIs libraries. The volcal commands will be acquired by the UI node and sent as *strings* to the *detected_word.py*. This *custom server script* will prosess the UI request by using the functions contained in the *vocal.py* script. The detected_word Nde will send two *integer* values corresponding to the required action back to the UI node. The response will be processed in the UI to send the corresponding goal to the correct ROS Action. Two other extra filed of the respons will describe to the UI The name of the acquired command and a quick description of the effect on the robot's motion. These two strings will be displayed ay time the user sends a new goal.

* `moving_control.py` & `moving_server.py`: These two nodes represent a custom *Action Client (moving_control)* - *Action Server (moving_server)* relationship inside the architecture. The Commands they process are related to the movemnt of the robot's base. Once the UI will send the appropriate message to the moving_control Node's topic, the *Action-Client* will send the required custom *Action-message* to the *moving_server*.The Action message we decided to exchange is composed by three fields:

	* `velocity`: This field represents the linear velocity [*m/s*] sent to the server. This value could change if the robot is asked to accelerate or slow down. 
	* `turn`: This value represents the angular velocity [*rad/s*] sent to the server.
	* `time`: This value represents the duration of the required action [*s*].

The *Action-server* Node will publish as a *Twist* message the linear and angular velocity for the selected ammount of time on the */cmd_vel* IAGo's topic. Inside the `detected_word.py` Node is also implemented a subscriber to the TIAGo */scan_raw* topic. This subscription will retrive a *LaseScasn* message containing all the information of the TIAGo's array of lasers embedded inside its base. This array Scans the surroundings infront of the robot. If anything is detected to be too close to the robot while it's moving the active *Action Goal* would get cancelled. The command towards the direction  of the detection will be disabled.

* `animation_node.py`: This is an *Action-Client* node sending goals to the *PlayMotion* Action. This package automatically enables executing simultaneous trajectories in multiple groups of joints by sending a simple *string* message in the *motion_name Action-message* field. Once the UI will send the appropriate message to the *animation_node* the client will send the corresponding request to the *Action-Server* and the Goal will be sent directly to the robot. This Action will trivially make TIAGo execute many pre-planned motions like waving, pinching its gripper, look at the surroundings and many others. All the allowed commands will be summerised at the beginning of the UI interface.

* `move_control_node.cpp`: This Node is Client to the *FollowJointTrajectory* Action. This action is capable of sending goal positions to the 7 joints of the TIAGo's manipulator using the filds *FollowJointTrajectory* goal message. Based on the cmmand recived from the UI, the code will iter decide to encrese or decrese the angualr position of the shulder and elbow joits of the manipulator. Each angle will encrese and decrese by the 0.8[*rad*] radiants. The angles will stop changing if the work-envelope of the manipulator is reached.

__Running the simulation__
----------------------

### Intalling 

The simulation can be run on __Ubuntu 18.04.2 LTS__ operative system with __ROS Melodic__ environment. You can follow [*this*](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) guide to configure the latter and install all the ROS packages related to TIAGo.
The voice recognizer can be installed by following the README of [*this*](https://github.com/Uberi/speech_recognition#readme) GitHub repository. Please note that this package only uses some functionalities ( __Python__, __PyAudio__ and __Google API Client Library for Python__), you don't need to install all the listed libraries.
This package can be installed by moving all the files into a new folder called "speech_rec" and put it into the "src" folder of your ROS workspace, then run:
```bash
$ catkin build speech_rec  #build the package
```

### Running

To Run the simulation it is needed to use the two following *launch files*:

* `tiago_gazebo.launch`: that will launch the **gazebo** simulation of the TIAGo Robot.

* `run.launch`: Which is a launch file we added to the *speech_rec* pakage. This file will launch all the previously mentioned nodes including the use of the *Xterm* terminal for the User Interface.

Run the following commands from the shell to activate all the nodes:

```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel

roslaunch final_assignment launchAll.launch
```

This kind of execution needs the Xterm terminal to be installed. If it's not already installed you can download it with the following shell command:

```bash

sudo apt-get install -y xterm
```


