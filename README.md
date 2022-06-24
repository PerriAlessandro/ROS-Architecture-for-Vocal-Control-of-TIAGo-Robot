__SOFAR Assignment - TIAGo Vocal Controller__ 
================================
__Aim of the project__
----------------------
The goal of this project is to build a simple software architecture to perform some basic control on
TIAGo robot through vocal commands. To be more precise, __Google Speech Recognition__ APIs have been used to recognize the vocal commands of the user. The keywords detected by TIAGo are many and can be subdivided into three main categories:
- `Motion of the base`: the robot is able to perform some basics movements of the base, such as going forward and backward, turn left and right. Moreover, a simple obstacle avoidance algorithm has been implemented.
- `Motion of the arms`: straight-forward additive motions such as increasing the position of the joints corresponding to the shoulder and the elbow.
- `Pre-defined motions`: based on
[play_motion](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion) package, TIAGo is now able to play predefined upper-body motions through vocal commands.

__Installation and how to run__
----------------------
The simulation can be run on __Ubuntu 18.04.2 LTS__ operative system with __ROS Melodic__ environment. You can follow [*this*](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) guide to configure the latter and install all the ROS packages related to TIAGo.
The voice recognizer can be installed by following the README of [*this*](https://github.com/Uberi/speech_recognition#readme) GitHub repository. Please note that this package only uses some functionalities ( __Python__, __PyAudio__ and __Google API Client Library for Python__), you don't need to install all the listed libraries.
This package can be installed by moving all the files into a new folder called "speech_rec" and put it into the "src" folder of your ROS workspace, then run:
```bash
$ catkin build speech_rec  #build the package
```
