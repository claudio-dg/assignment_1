Experimental Robotics Lab first assignment
================================
This repository contains the work for the first assignment of Experimenal Robotics Laboratory


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [Project structure](#project-structure)
* [Software Components](#software-components)
* [Behaviuor Presentation](#behaviuor-presentation)
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
	
In the same way it presents the ```Clients``` to ```/motion/planner``` and ```/motion/controller``` Services in order to call them respectively: to plan a random plan of waypoints and to move the robot throughout them.
	
Last it presents a client for the ```armor_interface_srv``` service to interact with the ontology with the ```aRMOR_api```.	
	
Therefore this node manages the overall behaviour of the robot by implementing a ```Finite States Machine``` that is made of 4 different states (```LoadOntology```, ```Decide```, ```Surveillance```, ```Recharging```), that are reached through 5 different transitions (```'loaded'```,```'decided'```,```'visited'```,```'low_battery'```,```'recharged'```), as shown in the following graph obtained with ```smach_viewer```.

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/FSM.png?raw=true" width="400" />
<p>

#### LoadOntology State: ####
	
This is the ```Init state``` of the FSM, within which it waits for the map to be loaded, therefore it simply uploads the ontology of the map by calling  helper's function ```"MY_loadOntolgy"```, and then it returns the ```'loaded'``` transition in order to move to the successive state i.e. ```Decide```.

```bash
class LoadOntology(smach.State):

    def __init__(self, my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper

        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        print('+++++ Executing state LoadOntology +++++')
        self._helper.MY_LoadOntology()
        print('Loading Ontology...')
        rospy.sleep(4)
        return 'loaded'	
```
#### Decide State: ####

In this state the robot chooses next move calling helper's function ```ChooseNextMove()``` and plans a path to reach such goal calling ```PlanToNext()```. It will put the results of these function calls into global variables shared among states, then returns ```'low_battery'``` transition if battery gets low during this phase (to move to ```Recharging``` state), otherwise returns ```'decided'``` transition to move to ```Surveillance``` state.
	
```bash
class Decide(smach.State):

    def __init__(self,my_helper):
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
 
    def execute(self, userdata):
        print(' +++++ Executing state Decide +++++')
        global chosen_location
        global path_planned

        try:
            # CHOOSE NEXT LOCATION
            chosen_location = self._helper.ChooseNextMove()
            if(chosen_location == 0):
                # low_battery battery occured
                return 'low_battery'
            else:
                # Room chosen successfully
                print("NEXT ROOM WILL BE --> ", chosen_location)
                # PLAN A PATH TO REACH NEXT LOCATION
                path_planned = self._helper.PlanToNext(self.check_battery)
                if(path_planned == 0):
                    # low_battery battery occured
                    return 'low_battery'
                else:
                    # path planned successfully
                    # print('path planned successfully -> ', path_planned)
                    print('PATH PLANNED SUCCESSFULLY')
                    return 'decided'
        finally:
            rospy.sleep(0.1)	
```
#### Surveillance State: ####

In this state the robot simulates time waste to reach the goal through the waypoints planned in ```Decide state```, calling helper's function  ```SimulateMotionTime```, then it actually moves to the next room (again received from ```Decide state```) calling helpers'function ```MoveToNext``` which manipulates the ontology. After that it applies the surveillance algorithm of the new room by calling helper's function ```Survey```. In the end it returns ```'low_battery'``` transition if battery gets low during these phases, (to move to ```Recharging``` state), otherwise returns ```'visited'``` transition to move back to ```'Decide'``` state.

```bash
class Surveillance(smach.State):
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
       
    def execute(self, userdata):
        print(' +++++ Executing state Surveillance +++++')
        # retrieve the location and path computed in'Decide' state from global variables
        global chosen_location
        global path_planned

        # SIMULATE GOAL REACHING WITH CONTROLLER
        motion_completed = self._helper.SimulateMotionTime(path_planned,self.check_battery)

        if(motion_completed == 0):
            # low battery occured
            return 'low_battery'
        else:
            # plan followed successfully
            self._helper.MoveToNext(chosen_location)
            # SURVEY LOCATION
            visited = self._helper.Survey()
            if(visited == 0):
                # low battery occured
                return 'low_battery'
            else:
                # Surveillance completed successfully
                print("SURVEILLANCE OVER.. ")
                rospy.sleep(1)
                return 'visited	
```
	
	
	
#### Recharging State: ####
	
Robot gets in this state when its battery gets low, here it will first move to reach the recharging station thanks to helper's function ```"GoToRechargingStation()"```, then it will start the recharging phase by calling this state in loop (through the ```low_battery``` transition) until the robot battery gets high. At that point it will return ```recharged``` transition to go to ```Decide``` state.
	
```bash
class Recharging(smach.State):
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
        self.time = 0
        self.station_reached = 0
	
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        print('+++++ Executing state Recharging +++++')

        # FIRST REACH THE RECHARGING STATION IN ORDER TO START THE RECHARGING PROCESS
        if(not self.station_reached):
            self.station_reached = self._helper.GoToRechargingStation()
            return 'low_battery'

        else:
            self._helper.mutex.acquire()
            try:
                if(self._helper.is_battery_low()):
                    # until battery is low
                    print("I AM RECHARGING THE BATTERY-- ELSAPSED TIME: ", str(int(self.time)), "seconds", end ='\r')
                    self.time += 1
                    rospy.sleep(1)
                     # repeat this state until battery is charged
                    return 'low_battery'
                else: # battery fully charged
                    print("BATTERY FULLY CHARGED AFTER : ", str(int(self.time)), "seconds") #print non funza
                    # reset variables
                    self.time = 0
                    self.station_reached = 0
                    return 'recharged'
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(1)
```
	
	
	
----------------------
### Controller node  : ###
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements an action server named ```motion/controller```. Here the modifications are mainly represented by some parameters changes, therefore the structure of the node is inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation

----------------------	
### Planner node  : ###
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements an action server named ```motion/planner```. Here the modifications are mainly represented by some parameters changes, therefore the structure of the node is inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation
----------------------
### Helper classes  : ###
As previously said, the script ```helper.py``` contains the implementation of two different helper classes: ```ActionClientHelper``` & ```Helper```. The first one is a class to simplify the implementation of a client for ROS action servers, and has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository in order to implement the planning and controlling action, so please check the linked repository for further documentation.
The second one instead is an helper class I made to simplify the interaction with the ontology through aRMOR, and therefore to define useful functions to be used in the Finite states machine script (```FSM.py```). Here you can find the definition of the subscriber to ```/state/battery_low``` topic to receive continuous information about the battery state previously named in ```FSM node```, as well as the definition of the```clients``` to ```/motion/planner``` and ```/motion/controller``` Services.






----------------------
### Robot-State node  : ### 
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements two ```services``` (i.e., ```state/set_pos```e and ```state/get_pose```) and a ```publisher``` (i.e., ```state/battery_low```) that runs on a separate thread. Here the modifications are mainly represented by some parameters changes related to the battery state, for which i set two different "timers" to set "charging/discharging" time, along with the elimination of some redundant part for my oroject. Therefore the structure of the node is pretty much inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation.


RIGUARDO helper.py, dopo l'introduzione generale, spiego solo i metodi chiave in sottocapitoletti, ad esempio --SPIEGO QUELLI CHE VENGONO CHIAMATI IN FSM!!!!!
	
"loadOntology...carica onto facendo query ecc ecc NOTA RIPETUTO 3 VOLTE PER BUG e infine la salva in file.
	
chooseNextMove --> implementa algoritmo di scelta (spiego algo: controllo ogni colta batteria, se non e scarica controllo urg ecc ecc)

poi boh quelli sulla batteria, poi go to recharging e boh
	
RIGUARDO FSM.PY MOSTRO LA MSF OTTENUTA

	
 

## Behaviuor Presentation
 
QUI SPIEGO QUINDI COSA FA PASSO PASSO IL MIO CODICE MOSTRANDO ANCHE GLI SCREEN DEL TERMINALE PER OGNI STEP	

 
 ## Limitations and Possible Improvements
 
The main limitations of this project are related to the current implementation of ```Planning``` and ```Controlling``` algorithms. Here in fact, the code written in  [planner.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/planner.py) and in [controller.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/controller.py), only allow to ```simulate``` these actions, without actually thinking about how to reach a real room in the environment, but instead simulating the time waste of this planning action along with the one required for actually moving along the planned path. 
	
Therefore **possible improvements** could involve the implementation of a better algorithm that actually takes into account a real environment in order to plan some real waypoints to reach a real position, while regarding the controller node, a better algorithm could take information from a real robot to actually control it along the planned path.
	
In addition to this the implementation of the ```robot battery``` could be further improved, since for now it is just represented by a boolean value changing randomly its value after some predefined time, so a better algoritmh could be used to discharge the robot accordingly with the ```travelled distance``` for instance, which would also avoid the robot to reach locations that are too far from its recharging station, that would prevent him to go back and recharge its battery. Moreover another limition about it is that, if we use this particular robot simulaton within a wider map, it could happen that robot may actually receive the "recharged" battery state before actually reaching the Recharging station, therefore the algorithm based on "travelled distance" previously named could help also in this case. 
 

