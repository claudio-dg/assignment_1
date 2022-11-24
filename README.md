Experimental Robotics Lab first assignment
================================
This repository contains the work for the first assignment of Experimenal Robotics Laboratory


Table of contents
----------------------

* [Introduction](#introduction)
* [Setup and Dependencies](#setup-and-dependencies)
* [Gazebo and Rviz Maps](#gazebo-and-rviz-maps)
* [Project structure and behaviour description](#project-structure-and-behaviour-description)
* [PseudoCode](#pseudocode)
* [RT2 Assignment](#rt2-assignment)


## Introduction

The goal of this assignment is to Develop a software architecture to control a robot, which is deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The 2D environment has to be produced making use of Armor_api to create an ontology, and should resemble the following map, made of 4 rooms and 3 corridors:

<p>
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/map.png?raw=true" width="850" />
<p>
	
	
<!--  immagine del rqt graph
<p>
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/assignment_1_rosgraph.png?raw=true" width="850" />
<p>
-->
Within this environment, the robot should:

1. start in the E location and waits until it receives the information to build the 
topological map,i.e., the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.
2. move in a new location, and should wait for some time before visiting another location. This 
behavior is repeated in a infinite loop. When robot’s battery is not low, it should move among locations with this policy:
- It should mainly stay on corridors,
– If a reachable room has not been visited for some time, it becomes ```URGENT``` and the robot should visit it.
3. When the robot’s ```battery is low```, it should go in the E location, and wait for some time before starting again with the above behavior

To define the behaviour of the robot we have to create a Finite States Machine using SMACH libraries





## Setup and Dependencies

This repository Contains all the useful files to run the scripts that i produced for this assignment.
To try it, it is sufficient to clone this repository in your ROS workspace: 

```bash
$ git clone https://github.com/claudio-dg/final_assignment.git
```

and then type the following command in the terminal to simultaneously launch all the necessary nodes through the **"launchFile"**:

```bash
$ roslaunch final_assignment final.launch

```
This Launch File has been made to make it easier to run the project , but if you like you can manually run every part of the project by launching the following launch files:

```bash
$ roslaunch final_assignment move_base.launch
$ roslaunch final_assignment simulation_gmapping.launch

```
To run the simulation environment and the move_base functions.

```bash
$ roslaunch final_assignment teleop.launch
$ roslaunch final_assignment my_scripts.launch
```
To run the teleop_Twist_keyboard node and my scripts produced for this assignment.

## Gazebo and Rviz Maps

The environment used for this assignment consists in the map illustrated (froma Gazebo view) in the following image:

<p>
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Gazebo.png?raw=true" width="450" />
<p>

Rviz instead gives another point of view of the same environment, that is from robot sensors' point of view: the robot, in fact, does not know from the beginning the full map he's in, but thanks to the laser sensors and the ```gmapping``` package he is capable of creating it.

<p>
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/Rviz.png?raw=true" width="400"/>
<p>
	
## Project structure and behaviour description

The project is based on the ROS scheme that is shown in the following graph:

<p align="center">
<img src="https://github.com/claudio-dg/final_assignment/blob/main/images/final_assign_rosgraph.png?raw=true" width="900"  />
<p>
 
The ROS package of the project is called ```"final_assignment"```, it exploits two already given packages: ```slam_gmapping```, which opens the environment and allows the robot to create a map of what sorrounds him, and ```move_base```, which requires a goal to be sent to the topic ```move_base/goal``` in order to make the robot move towards it.
	
In addition to these i created two nodes contained in ```src``` folder named ```InputConsole``` and ```controller```; as the name suggests the first one is encharged of taking user's inputs to select the desired behaviour of the robot, while the second one manages the consequences of user's request by communicating with other nodes, for instance by sending the goal's coordinates to ```move_base/goal``` with a msg of type :```move_base_msgs/MoveBaseActionGoal```.
The communication between my two nodes is implemented through a ```Publish/Subscribe``` architecture using two different topics ```MY_topic_teleop```  & ```MY_topic_send_goal```: in this way I made a structure in which the input given by the user determines which callback is going to be called in the controller node, so that the "async structure" required by this assignment was possible.
	
- Regarding point 1) I used the ```\move_base\feedback``` topic to retreive information about robot's status such as the current position or the time counter: thanks to these two pieces of information I implemented an algorithm to state whether the goal was reached or not (considering an approximation error due to the fact that the robot seemed to get really close to the goal but never reaching its exact coordinates), and a TIMEOUT, so that if the robot doesen't reach the goal in Time it is considered unreachable and will be canceled by sending a msg to ```\move_base\cancel``` topic
		
- Regarding points 2) and 3) of the assignment I remapped an already existing topic (```teleop_twist_keyboard```) so that instead of publishing directly on ```cmd_vel``` it publishes on my personal topic ```myRemapped_cmd_vel```: by doing this I manage to consider the velocities published by this topic only when required, that is when the user selected mode 2) or 3), furthermore it allowed me to add the collision avoidance functionality needed for the third part of the assignment. 


 ### Behaviour description  : ### 

After having launched all the required launch files Gazebo and Rviz environments will open, along with 3 different terminals:
* ```Input Console``` : in which you can select what to do and that will show the following user interface:
```bash
***********THIS IS THE INPUT CONSOLE***********

Which Action do you want to use to move the robot?
ENTER 'c' to cancel last given goal
ENTER '1' to send a new goal to the robot
ENTER '2  to manually drive the robot
ENTER '3' to manually drive the robot WITH assisted collision avoidance
ENTER 'q' to terminate this node
```

* ```Controller Console``` : that will show some useful real-time info depending on the modality selected in the input console, such as the elapsed time since the goal was given or some notifications to inform the user tha a certain direction will probably cause a collision.
* ```TeleopTwist Keyboard Console``` : in which the user can insert commands to manually drive the robot that will only be read if modality 2) or 3) were previously selected through the input console. 
 

	
	
 ## Pseudocode
 
 To reproduce the behaviour previously described i wrote 2 C++ programms contained in the ```src``` folder:
 - input_console.cpp
 - controller.cpp 

### Input_console.cpp  : ###


```bash
initialize node
initialize necessary publishers and subscribers
Use continuous callback from clock topic
	
	print User Interface menu
	wait for a keyboard input and put it into a variable "input"
	switch(input)
	    case 'c':
	    	publish on move_base/cancel topic to cancel last given goal
		publish a flag msg on MY_topic_send_goal to reset variables in controller
		print that goal has been canceled
		break;
	    case '1':
		print autonomous driving introduction
                ask for goal coordinates
	    	read goal coordinates from the user
                publish them on MY_topic_send_goal topic to notify the controller
		print sent coordinates
	    	break;
	    case '2':
	    	print manual driving introduction
		print a message to tell the user to look at teleopkeyboard console
		publish a boolean msg equal to 0 on MY_topic_teleop to inform the controller that manual driving was selected
		break;           
	    case '3':
	    	print ASSISTED manual driving introduction
		print a message to tell the user to look at teleopkeyboard console
		publish a boolean msg equal to 1 on MY_topic_teleop to inform the controller that ASSISTED manual driving was selected
		break;
	    case 'q':
		print an exiting message
		exit the program
	    default:
	    	print an error for a wrong command inserted
	    	break;
	
```
	
### Controller.cpp  : ###	

```bash
initialize node
initialize necessary publishers and subscribers
print controller console introduction	
	//the MAIN loops with ros::SpinOnce whilethe program isn't killed
	//check for global flag values
	
	if user asked for manual drive
		take velocity from teleopKeyboard contained into myRemapped_cmd_vel topic
		publish it on cmd_vel topic to make the robot move
	if user asked for ASSISTED manual drive
		call assistedMovement function**
	if user didn't ask for these 2 modalities, wait for other callbacks to be called
	
	//**assitedMovement function:
		check distances received from laser sensors
		if there is an obstacle in front of the robot
			check for nearest obstacles at his sides
			if nearest obstacle is at his right
				turn left a bit
				print feedback to the user
			if nearest obstacle is at his left
				turn right a bit
				print feedback to the user
			set flag changedVel to 1 to state that the direction has been modified
			sleep 0.3 seconds
		if the direction has been modified
			stop the robot
		else
			clear the console
			take velocity from teleopKeyboard contained into myRemapped_cmd_vel topic
			publish it on cmd_vel topic to make the robot move
	
//basing on which msg is received from input cosole, different callbacks are executed
	if 'c' or '1' were inserted in input cosole  "myCallback" is executed
		
		reset manual drive global flag value
		put goal coordinates in a global variable
		if msg contains flag value for "goal canceled"
			clear the terminal
			print that goal has been canceled by user
			set "canceled" global flag value to 1
		else
			reset global flags variables
			publish goal on  move_base/goal topic
			print that goal has been published
	return
	
	if '2' or '3' were inserted in input cosole "TeleopCallback" is executed
		
		put msg's boolean value in the global flag variable "manualdrive"
		cancel possibly existing goals
		reset velocities to 0
		if msg asked for manual drive
			print manual driving introduction
			print a message to tell the user to look at teleopkeyboard console
		else if msg asked for ASSISTED manual drive
			print ASSISTED manual driving introduction
			print a message to tell the user to look at teleopkeyboard console
	
//last 2 callbacks (myCmdCallback & CurrentPositionCallback) are called respectively when user inserts command on teleopKeyboard console and when the robot status changes
	
	//myCmdCallback
	put the received vel in a global variable
	reset changedVel flag each time a new command is inserted
	
	
	//CurrentPositionCallback
	execute the callback only if goal wasn't reached yet (goal_reached==0)
	take current position coordinates from /move_base/feedback topic
	if the status has changed just now (firstTime==1)
		take current time as Starting time
		reset firstTime flag value
	keep updating current time
	compute elapsed time since the goal was given
	print info about goal coordinates and time elapsed
	compute Error between current position and Goal position
	if error is small enough
		set goal_reached flag to TRUE
	if the timeout is over and the goal hasn't been reached
		cancel the goal 
		print that goal has been canceled
	else if  goal has been reached before timeout was over
		clear console
		print that goal was reached
```


## RT2 Assignment
 
For this assigment we had to : 

1. Document our code using Doxygen. 
	The result of this part is contained in the ```Docs``` Folder.
	 Please clone the repository and open ```index.html``` with a browser to see the documentation page.
2. create a jupyter notebook to replace the user interface and to show info about the robot through real time plots. 
	The result of this part is contained in the ```Assignment_notebook.ipyn``` file. Before ruuning the notebook please make sure to run 
	```bash
	$ roslaunch final_assignment final.launch

	```
3. make a statistical analysis on the first assignment of the course.
	The results of this part are contained in a .pdf file and a matlab script that are sent separately to the professor as attachment to the mail.
	

	

 
 
 
 
 

