Experimental Robotics Lab first assignment
================================
This repository contains the work for the first assignment of Experimenal Robotics Laboratory


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [Project structure](#project-structure)
* [Software Components](#software-components)
* [Behaviuor Description](#behaviuor-description)
* [Limitations and Possible Improvements](#limitations-and-possible-improvements)


## Introduction

The goal of this assignment is to Develop a software architecture to control a robot, which is deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The 2D environment has to be produced making use of Armor_api to create an ontology, and should resemble the following map, made of 4 rooms and 3 corridors:

<p><p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/map.png?raw=true" width="400" />
<p>
	
	
<!--  
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

The project is based on the ROS scheme that is shown in the following ```rqt_graph```:

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/assignment_1_rosgraph.png?raw=true" width="850" />
<p>
 
This repository contains a ROS package called ```"assignment_1"``` that includes the following resources:

- [CMakeList.txt](https://github.com/claudio-dg/assignment_1/blob/main/CMakeLists.txt): File to configure this package.
- [package.xml](https://github.com/claudio-dg/assignment_1/blob/main/package.xml): File to configure this package.
- [Docs/](https://github.com/claudio-dg/assignment_1/tree/main/Docs): folder containing ```Doxygen documentation``` of the package
- [images/](https://github.com/claudio-dg/assignment_1/tree/main/images): folder containing images and graphs used within this [README](https://github.com/claudio-dg/assignment_1/blob/main/README.md).
- [scripts/](https://github.com/claudio-dg/assignment_1/tree/main/scripts): It contains the implementation of each software components produced for this project.
	
	* [FSM.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/FSM.py): contains the implementation of the SMACH Finite States Machine
	
	* [controller.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/controller.py): It is a dummy implementation of a motion controller. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [helper.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/helper.py): It contains the implementation of two helper classes for ROS actions and for interfacing the Ontology through aRMOR_api
	
	* [planner.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/planner.py): It is a dummy implementation of a motion planner. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [robot_state.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/robot_state.py): It implements the robot state including: current position, and battery level.
- [launch/](https://github.com/claudio-dg/assignment_1/tree/main/launch): It contains the launch file to start the simulation
	
	* [start_simulation.launch](https://github.com/claudio-dg/assignment_1/blob/main/launch/start_simulation.launch): launch file of the project
	
 
## Software Components

Here are shown the details of each software component implemented in this repository and contained in the ```scripts/``` folder.
### FSM node: ###
	
This node is "indirectly" subscribed to ```/state/battery_low``` topic to receive continuous information about the battery state, in the sense that the subscriber is not directly initialised within the ```"FSM.py"```script, rather it is obtained through the functions defined in the ```helper.py``` that gets called in each class of this node.
	
In the same way it presents the ```CLients``` to ```/motion/planner``` and ```/motion/controller``` Services in order to call them respectively: to plan a random plan of waypoints and to move the robot throughout them.
	
Last it presents a client for the ```armor_interface_srv``` service to interact with the ontology with the ```aRMOR_api```.	
	
Therefore this node manages the overall behaviour of the robot by implementing a ```Finite States Machine``` that is made of 4 different states (```LoadOntology```, ```Decide```, ```Surveillance```, ```Recharging```), that are reached through 5 different transitions (```'loaded'```,```'decided'```,```'visited'```,```'low_battery'```,```'recharged'```), as shown in the following graph obtained with ```smach_viewer```.

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/FSM.png?raw=true" width="400" />
<p>

#### LoadOntology State: ####
	aa
#### Decide State: ####
#### Surveillance State: ####
#### Recharging State: ####
	
	
	
### Controller node  : ###
	
### Planner node  : ###
	
### Helper classes  : ###
	
### Robot-State node  : ### 
	credits to arch ma ho leggermente cambiato in particolare eliminando parti non utili alla mia applicazione sttando due range diversi da cui scegliere il tempo necessario a scarica o carica
	
	
spiego come radme del prof in arch_scheleton, i singoli nodi cosa fanno (sub/publish ecc + spiegazione del codice!) 
	
RIGUARDO helper.py, dopo l'introduzione generale, spiego solo i metodi chiave in sottocapitoletti, ad esempio 
	
"loadOntology...carica onto facendo query ecc ecc NOTA RIPETUTO 3 VOLTE PER BUG e infine la salva in file.
	
chooseNextMove --> implementa algoritmo di scelta (spiego algo: controllo ogni colta batteria, se non e scarica controllo urg ecc ecc)

poi boh quelli sulla batteria, poi go to recharging e boh
	
RIGUARDO FSM.PY MOSTRO LA MSF OTTENUTA

	
 

## Behaviuor Description
 
QUI SPIEGO QUINDI COSA FA PASSO PASSO IL MIO CODICE MOSTRANDO ANCHE GLI SCREEN DEL TERMINALE PER OGNI STEP	

 
 ## Limitations and Possible Improvements
 
The main limitations of this project are related to the current implementation of ```Planning``` and ```Controlling``` algorithms. Here in fact, the code written in  [planner.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/planner.py) and in [controller.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/controller.py), only allow to ```simulate``` these actions, without actually thinking about how to reach a real room in the environment, but instead simulating the time waste of this planning action along with the one required for actually moving along the planned path. 
	
Therefore **possible improvements** could involve the implementation of a better algorithm that actually takes into account a real environment in order to plan some real waypoints to reach a real position, while regarding the controller node, a better algorithm could take information from a real robot to actually control it along the planned path.
	
In addition to this the implementation of the ```robot battery``` could be further improved, since for now it is just represented by a boolean value changing randomly its value after some predefined time, so a better algoritmh could be used to discharge the robot accordingly with the ```travelled distance``` for instance, which would also avoid the robot to reach locations that are too far from its recharging station, that would prevent him to go back and recharge its battery. Moreover another limition about it is that, if we use this particular robot simulaton within a wider map, it could happen that robot may actually receive the "recharged" battery state before actually reaching the Recharging station, therefore the algorithm based on "travelled distance" previously named could help also in this case. 
 

