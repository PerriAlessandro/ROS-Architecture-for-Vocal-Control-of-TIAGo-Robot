#  [SofAR](https://corsi.unige.it/off.f/2020/ins/43589) , [Robotics Engineering](https://courses.unige.it/10635) ([UNIGE](https://unige.it/it/)) : Assignment
## [TIAGo](https://pal-robotics.com/robots/tiago/) [Vocal Controller](https://github.com/Uberi/speech_recognition)
### Authors: [Matteo Carlone](https://github.com/MatteoCarlone), [Fabio Conti](https://github.com/Fabioconti99), [Alessandro Perri](https://github.com/PerriAlessandro)
### Professor. [Fulvio Mastrogiovanni](https://rubrica.unige.it/personale/UkNHWFhr), [Simone Macciò](https://rubrica.unige.it/personale/UUNAWFho)



__Aim of the project__
----------------------
The goal of this project is to build a simple [ROS](https://www.ros.org) software architecture to perform some basic control on the [PAL Robotics](https://github.com/pal-robotics) TIAGo robot through vocal commands. To be more precise, __Google Speech Recognition__ APIs have been used to recognize the vocal commands of the user. The keywords detected by TIAGo are many and can be categorized into three main groups:
- `Motion of the base`: the robot is able to perform some basics movements of the base, such as going forward and backward, turn left and right. Moreover, a simple obstacle avoidance algorithm has been implemented.
- `Motion of the arms`: straight-forward additive motions such as increasing the position of the joints corresponding to the shoulder and the elbow with respect to their zero position.
- `Pre-defined motions`: based on [play_motion](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion) package, TIAGo is now able to play predefined upper-body motions through vocal commands.
These commands sets correspond to the three different **ROS Actions** deployed by the software to animate the robot exploiting different topics and communication paths.


__Running the simulation__
----------------------

### Installing 

The simulation can be run on __Ubuntu 18.04.2 LTS__ operative system with __ROS Melodic__ environment. You can follow [*this*](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) guide to configure the later and install all the ROS packages related to TIAGo.
The voice recognizer can be installed by following the README of [*the speech recognition*](https://github.com/Uberi/speech_recognition#readme) GitHub repository. Please note that this package only uses some functionalities ( __Python__, __PyAudio__ and __Google API Client Library for Python__) of the voice recognizer, you don't need to install all the listed libraries.
This package can be installed by moving all the files into a new folder called "speech_rec" and put it into the "src" folder of your ROS workspace, then run:

```bash
$ catkin build speech_rec  #build the package
```

### Running

To Run the simulation it is needed to use the two following *launch files*:

* `tiago_gazebo.launch`: that will launch the **gazebo** simulation of the TIAGo Robot.

* `run.launch`: which is a launch file we added to the *speech_rec* package. This file will launch all the previously mentioned nodes including the use of the *Xterm* terminal for the User Interface.

Run the following commands from the shell to activate all the nodes:

```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel

roslaunch speech_rec run.launch
```

This kind of execution needs the Xterm terminal to be installed. If it's not already installed you can download it with the following shell command:

```bash

sudo apt-get install -y xterm
```


__Description of the Software architecture__
----------------------
To create the software achitecture for commanding the TIAGo robot we developed a **ROS Package** called *speech_rec*. Inside the folder we developed multiple **ROS Nodes** and scripts to propose the most modular and schematic solution to the given problem.
The following list summarises the role of each node inside the achitecture:

* `UI.py` & `detected_word.py`: The User Interface script will recognize the vocal commands of the user thanks to the [*voice recognizer*](https://github.com/Uberi/speech_recognition) which exploits the __Google Speech Recognition__ APIs libraries. The vocal commands will be acquired by the UI Node and sent as *strings* to the `detected_word.py` server-node in a *Client-Server* type of communication. 
The *custom server script* will process the UI request by using the functions contained in the `vocal.py` script. 
The detected_word Node will send two *integer* values corresponding to the required action back to the UI node. The response will be processed in the UI to send the corresponding goal to the correct ROS Action. Two other extra fileds (*name, info*) of the response will send back to the UI The name of the acquired command and a quick description of the effect on the robot's motion. These two strings will be displayed inside The UI interface anytime the user sends a new goal. At the beginning of the UI terminal all the possible commands will be listed for the user to read and execute. The user will have to pronounce first the *"Alexa"* keyword followed by the **Command** willing to be executed by TIAGo.

* `moving_control.py` & `moving_server.py`: These two nodes represent a custom *Action Client (moving_control)* - *Action Server (moving_server)* relationship inside the architecture. The Commands they process are related to the movemnt of the robot's base. Once the UI will send the appropriate message to the moving_control Node's topic, the *Action-Client* will send the required custom *Action-message* to the *moving_server*. The Action message we decided to exchange is composed by three fields:

	* `velocity`: This field represents the linear velocity [*m/s*] sent to the server. This value could change if the robot is asked to accelerate or slow down. 
	* `turn`: This value represents the angular velocity [*rad/s*] sent to the server.
	* `time`: This value represents the duration of the required action [*s*].

The *Action-server* Node will publish as a `Twist` message the linear and angular velocity for the selected amount of time on the `/cmd_vel` TIAGo's topic. Inside the `detected_word.py` Node there's also implemented a subscriber to the TIAGo `/scan_raw` topic. This subscription will retrieve a *LaseScan* message containing all the information of the TIAGo's array of lasers embedded inside its base. This array scans the surroundings in front of the robot. If anything is detected to be too close to the robot while it is moving the active *Action Goal* would get cancelled. The command towards the direction of the detection will be disabled until the obstacle is not anymore detected within the critical distance.

* `animation_node.py`: This is an *Action-Client* node sending goals to the *PlayMotion* Action. This package automatically enables executing simultaneous trajectories in multiple groups of joints by sending a simple *string* message in the *motion_name Action-message* field. Once the UI will send the appropriate message to the *animation_node* the client will send the corresponding request to the *Action-Server* and the Goal will be sent directly to the robot. This Action will trivially make TIAGo execute many pre-planned motions like waving, pinching its gripper, look at the surroundings and many others. All the allowed commands will be summarised at the beginning of the UI interface.

* `move_control_node.cpp`: This Node is Client to the *FollowJointTrajectory* Action. This action is capable of sending goal positions to the 7 joints of the TIAGo's manipulator using the fields of *FollowJointTrajectory* goal message. Based on the command received from the UI, the code will either decide to encrease or decrease the angular position of the shoulder and elbow joints of the manipulator. Each angle will encrease and decrease by the 0.8[*rad*] radiants. The angles will stop changing if the work-envelope of the manipulator is reached.



__Architecture's UML graph__
----------------------
![UML SOFAR](https://user-images.githubusercontent.com/91262561/178252817-a2c846bf-97cb-4a2b-afbb-1bdc70939368.png)


