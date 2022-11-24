Experimental Robotics Lab first assignment
================================
This repository contains the work for the first assignment of Experimenal Robotics Laboratory


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [Project structure](#project-structure)
* [Software Components](#software-components)
* [PseudoCode](#pseudocode)
* [RT2 Assignment](#rt2-assignment)


## Introduction

The goal of this assignment is to Develop a software architecture to control a robot, which is deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The 2D environment has to be produced making use of Armor_api to create an ontology, and should resemble the following map, made of 4 rooms and 3 corridors:

<p><p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/map.png?raw=true" width="400" />
<p>
	
	
<!--  immagine del rqt graph
<p>
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/assignment_1_rosgraph.png?raw=true" width="850" />
<p>
-->
Within this environment, the robot should:

1. start in the E location and waits until it receives the information to build the 
```topological map```, i.e. the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.
2. move in a new location, and should wait for some time before visiting another location. This 
behavior is repeated in a infinite loop. When robot’s battery is not low, it should move among locations with this ```policy```:
- It should mainly stay on corridors,
- If a reachable room has not been visited for some time, it becomes ```URGENT``` and the robot should visit it.
3. When the robot’s ```battery is low```, it should go in the E location, and wait for some time before starting again with the above behavior

<!-- To define the behaviour of the robot we have to create a Finite States Machine using SMACH libraries -->





##  Dependencies and Setup

In order to run correctly the project of this repository, some important dependencies have to be taken into account, therefore please make sure to have the following packages already installed in your ```ros_workspace```:
- [arch_skeleton](https://github.com/buoncubi/arch_skeleton): from which I retrieved useful [actions](https://github.com/buoncubi/arch_skeleton/tree/main/action), [messages](https://github.com/buoncubi/arch_skeleton/tree/main/msg) [services](https://github.com/buoncubi/arch_skeleton/tree/main/srv) and [utilities](https://github.com/buoncubi/arch_skeleton/tree/main/utilities/arch_skeleton)	to implement the planner and controller of this repository. Please note that credits for the planner and controller also go this repository, since they have been implemented starting from the example there shown and slightly adapting them to my needs.
- [topological_map](https://github.com/buoncubi/topological_map): from which I retrieved the [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl), to use it as starting point for creating my ```ontology``` of the environment for the assignment, which will be created in run time and saved as a .owl file in the ```topological_map``` folder.
- [aRMOR](https://github.com/EmaroLab/armor): which is necessary to use an OWL ```ontology``` and the related ```reasoner``` within ROS. Particularly useful is the [armor_py_api](https://github.com/EmaroLab/armor_py_api), which simplifies the calls to aRMOR, but can only be used from a python-based ROS node, and it allows to interact with ontologies with three types of operations: manipulations, queries, and ontology management.

In the end, the Finite States Machine here implemented is based on [SMACH](http://wiki.ros.org/smach) libraries.
	
When all these are correctly installed, to try this repository it is necessary to: clone it in your ROS workspace: 

```bash
$ git clone https://github.com/claudio-dg/assignment_1.git
```

then type the following commands in the terminal to make sure to launch the rosmaster as well as the aRMOR service:

```bash
$ Roscore &
$ rosrun armor execute it.emarolab.armor.ARMORMainService

```
Please note that after building the package for the first time it may be required to go to the project directory and run the following command to correctly use aRMOR

```bash
$ ./gradlew deployApp
```
	
After that you can type the following command in the terminal to simultaneously launch all the necessary nodes through the [launchFile](https://github.com/claudio-dg/assignment_1/tree/main/launch):

```bash
$ roslaunch assignmnent_1 start_simulation.launch
```

## Project structure

The project is based on the ROS scheme that is shown in the following graph:

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/assignment_1_rosgraph.png?raw=true" width="850" />
<p>
 
<!-- 
The ROS package of the project is called ```"final_assignment"```, it exploits two already given packages: ```slam_gmapping```, which opens the environment and allows the robot to create a map of what sorrounds him, and ```move_base```, which requires a goal to be sent to the topic ```move_base/goal``` in order to make the robot move towards it.
	
In addition to these i created two nodes contained in ```src``` folder named ```InputConsole``` and ```controller```; as the name suggests the first one is encharged of taking user's inputs to select the desired behaviour of the robot, while the second one manages the consequences of user's request by communicating with other nodes, for instance by sending the goal's coordinates to ```move_base/goal``` with a msg of type :```move_base_msgs/MoveBaseActionGoal```.
The communication between my two nodes is implemented through a ```Publish/Subscribe``` architecture using two different topics ```MY_topic_teleop```  & ```MY_topic_send_goal```: in this way I made a structure in which the input given by the user determines which callback is going to be called in the controller node, so that the "async structure" required by this assignment was possible.
	
- Regarding point 1) I used the ```\move_base\feedback``` topic to retreive information about robot's status such as the current position or the time counter: thanks to these two pieces of information I implemented an algorithm to state whether the goal was reached or not (considering an approximation error due to the fact that the robot seemed to get really close to the goal but never reaching its exact coordinates), and a TIMEOUT, so that if the robot doesen't reach the goal in Time it is considered unreachable and will be canceled by sending a msg to ```\move_base\cancel``` topic
		
- Regarding points 2) and 3) of the assignment I remapped an already existing topic (```teleop_twist_keyboard```) so that instead of publishing directly on ```cmd_vel``` it publishes on my personal topic ```myRemapped_cmd_vel```: by doing this I manage to consider the velocities published by this topic only when required, that is when the user selected mode 2) or 3), furthermore it allowed me to add the collision avoidance functionality needed for the third part of the assignment. 

-->
 
## Software Components

spiego come radme del prof in arch_scheleton, i singoli nodi cosa fanno (sub/publish ecc + spiegazione del codice!) 
	
RIGUARDO helper.py, dopo l'introduzione generale, spiego solo i metodi chiave in sottocapitoletti, ad esempio 
	
"loadOntology...carica onto facendo query ecc ecc NOTA RIPETUTO 3 VOLTE PER BUG e infine la salva in file.
	
chooseNextMove --> implementa algoritmo di scelta (spiego algo: controllo ogni colta batteria, se non e scarica controllo urg ecc ecc)

poi boh quelli sulla batteria, poi go to recharging e boh
	
RIGUARDO FSM.PY MOSTRO LA MSF OTTENUTA
<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/FSM.png?raw=true" width="400" />
<p>
	
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
	

	

 
 
 
 
 

