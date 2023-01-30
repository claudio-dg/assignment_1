Experimental Robotics Lab first assignment
================================
This branch of the repository contains the work for the first assignment of Experimental Robotics Laboratory, with the changes applied for the second part of the assignment, in order to apply the project to a real robot in a simulated environment.

#### Author: Claudio Del Gaizo, S4696649

#### mail: cdg9914@gmail.com


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [Project structure](#project-structure)
* [Software Components and code description](#software-components-and-code-description)
* [Behaviuor Presentation](#behaviuor-presentation)
* [Limitations and Possible Improvements](#limitations-and-possible-improvements)


## Introduction

The goal of this assignment is to develop a software architecture to control a robot, which is deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The 2D environment has to be produced making use of Armor_api to create an ontology, and should resemble the following map, made of 4 rooms and 3 corridors:

<p><p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/map.png?raw=true" width="400" />
<p>
	
	
<!--  
-->
Within this environment, the robot should:

1. start in the E location and wait until it receives the information to build the 
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

- [CMakeList.txt](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/CMakeLists.txt): File to configure this package.
- [package.xml](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/package.xml): File to configure this package.
- [Docs/](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes/Docs): folder containing ```Doxygen documentation``` of the package
- [images/](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes/images): folder containing images and graphs used within this [README](https://github.com/claudio-dg/assignment_1/blob/main/README.md).
- [scripts](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes/scripts): It contains the implementation of each software components produced for this project.
	
	* [FSM.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/FSM.py): contains the implementation of the SMACH Finite States Machine
	
	* [controller.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/controller.py): It is a dummy implementation of a motion controller. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [helper.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/helper.py): It contains the implementation of two helper classes for ROS actions and for interfacing the Ontology through aRMOR_api
	
	* [planner.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/planner.py): It is a dummy implementation of a motion planner. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [robot_state.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/robot_state.py): It implements the robot state including: current position, and battery level.
- [launch/](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes/launch): It contains the launch file to start the simulation
	
	* [start_simulation.launch](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/launch/start_simulation.launch): launch file of the project
	
 
## Software Components and code description
	
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
#### MODIFIED FOR ASSIGNMENT 2 ####
	
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


Here follows the code explanation of the main functions of this class, that are the ones that get called directly by ```FSM node```:

#### MY_LoadOntology(self)  : ####
This function creates the map adding all the locations, doors and required properties to the ontology ```/topological_map.owl``` by using the "manipulation" tool of aRMOR_api client, then saves the result in a new file named ```MY_topological_map.owl```. Please note that here, due to some strange bugs, it has been necessary to have this function repeating the same actions 3 times with a while loop, because the manipulations were not correctly applied in the first 2 iterations, and therefore the reasoner was not able of correctly reasoning about the ontology.

```bash
def MY_LoadOntology(self):
        i = 1 
        done_once = 0 # to load timestamps only once
        while i <=3:

            # ADD ALL OUR AXIOMS
            if(client.manipulation.add_ind_to_class("R1", "LOCATION") and i == 3): #mettere if per checcare
                print("Added R1 to LOCATION")
            if(client.manipulation.add_ind_to_class("R2", "LOCATION") and i == 3):
                print("Added R2 to LOCATION")
            if(client.manipulation.add_ind_to_class("R3", "LOCATION")and i == 3):
                print("Added R3 to LOCATION")
            if(client.manipulation.add_ind_to_class("R4", "LOCATION")and i == 3):
                print("Added R4 to LOCATION")
            if(client.manipulation.add_ind_to_class("C1", "LOCATION")and i == 3):
                print("Added C1 to LOCATION")
            if(client.manipulation.add_ind_to_class("C2", "LOCATION")and i == 3):
                print("Added C2 to LOCATION")
            if(client.manipulation.add_ind_to_class("E", "LOCATION")and i == 3):
                print("Added E to LOCATION")
            if(client.manipulation.add_ind_to_class("D1", "DOOR")and i == 3):
                print("Added D1 to DOOR")
            if(client.manipulation.add_ind_to_class("D2", "DOOR")and i == 3):
                print("Added D2 to DOOR")
            if(client.manipulation.add_ind_to_class("D3", "DOOR")and i == 3):
                print("Added D3 to DOOR")
            if(client.manipulation.add_ind_to_class("D4", "DOOR")and i == 3):
                print("Added D4 to DOOR")
            if(client.manipulation.add_ind_to_class("D5", "DOOR")and i == 3):
                print("Added D5 to DOOR")
            if(client.manipulation.add_ind_to_class("D6", "DOOR")and i == 3):
                print("Added D6 to DOOR")
            if(client.manipulation.add_ind_to_class("D7", "DOOR")and i == 3):
                print("Added D7 to DOOR")


            # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
            client.manipulation.disj_inds_of_class("LOCATION")
            client.manipulation.disj_inds_of_class("DOOR")

            # ADD PROPERTIES TO OBJECTS
            # Distinction between rooms and corridors
            client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
            client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
            client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
            client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
            client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
            client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
            client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")


                # Distinction between individuals
            client.call('DISJOINT', 'IND', 'CLASS', ["R1","R2","R3","R4", "C1","C2", "D1", "D2", "D3", "D4", "D5", "D5", "D7"]) #IOOOOOOOOO

                # INITIALIZE ROBOT POSITION
            client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
                # ADD timestamps
            if(done_once == 0):
                client.manipulation.add_dataprop_to_ind("visitedAt","R1",'Long',str(int(time.time())))
                client.manipulation.add_dataprop_to_ind("visitedAt","R2",'Long',str(int(time.time())))
                client.manipulation.add_dataprop_to_ind("visitedAt","R3",'Long',str(int(time.time())))
                client.manipulation.add_dataprop_to_ind("visitedAt","R4",'Long',str(int(time.time())))
                done_once = 1

            # APPLY CHANGES
            client.utils.apply_buffered_changes()
            # client.utils.sync_buffered_reasoner()

            # SAVE AND EXIT
            client.utils.save_ref_with_inferences("/root/ros_ws/src/topological_map/MY_topological_map.owl")

            i +=1  #increase counter
```

#### ChooseNextMove(self)  : ####
This function implements the algorithm with which the robot decides what location to visit next. First it calls the reasoner to reson about the ontology and to updare robot timestamp (this is necessary to see when non visited rooms become urgent), then it querys the ontology to know what are the rooms that robot can reach and what are the currently urgent rooms. After that it starts if/elif/else statement in which: it checks the battery state in order to interrupt this phase in case robot gets low battery, otherwise the robot will:
	* randomly choose a URGENT reachable room (if any)
	* else it will randomly choose one of the reachable rooms giving preference to corridors (if any)
To implement this "simple" algorithm, many steps are actually necessary, and they are shown here below:

```bash
def ChooseNextMove(self):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()

            reachable_list = []
            urgents_list = []
            possible_corridors = []
            possible_rooms = []

            # ASK FOR REACHABLE ROOMS
            reachable_list = self.GetReachableRooms()
            print("Robot can reach ->",reachable_list)

            # ASK FOR EXISTING URGENT ROOMS
            urgents_list = self.GetUrgentRooms()
            print('Urgent Rooms ->', urgents_list)

            if(self.is_battery_low()):
                # if low_nattery occured while deciding, stop the process and pass to recharging state
                print("### LOW BATTERY WHILE DECIDING NEXT LOCATION ###")
                return 0

            elif not urgents_list:
                # else if the are NO URGENT ROOMS, move randomly giving preference to corridors
                for reachable_room in reachable_list:
                    if self.IsCorridor(reachable_room):
                        possible_corridors.append(reachable_room)
                    else:
                        possible_rooms.append(reachable_room)
                if(not possible_corridors):
                    # if there are NOT reachable corridors, choose a random reachable room
                    next_room = random.choice(possible_rooms)
                    print("THERE ARE NO URGENT ROOMS NOR CORRIDOR -> chose ",next_room ,"among -> ", possible_rooms)
                else:
                    # if there are reachable corridors choose one of them randomly
                    next_room = random.choice(possible_corridors)
                    print("THERE ARE NO URGENT ROOMS -> chose",next_room ," among -> ", possible_corridors)
                return next_room

            else:
                # else if the are URGENT ROOMS, check if they are reachable
                for urgent_room in urgents_list:
                    if(urgent_room in reachable_list): 
                        possible_rooms.append(urgent_room)
                if(possible_rooms):
                    # if there are reachable urgent rooms choose one of them randomly
                    next_room = random.choice(possible_rooms)
                    print("THERE ARE REACHABLE URGENT ROOMS-> chose ",next_room ,"among -> ", possible_rooms)
                    return next_room

                for reachable_room in reachable_list:
                    # else give preference to corridors if any
                    if self.IsCorridor(reachable_room):
                        possible_corridors.append(reachable_room)
                    else:
                        possible_rooms.append(reachable_room)
                if(not possible_corridors):
                    # if there are NOT reachable corridors, choose a random reachable room
                    next_room = random.choice(possible_rooms)
                    print("THERE ARE NO REACHABLE URGENT ROOMS NOR CORRIDORS,  chose -> ",next_room ,"among -> ", possible_rooms)
                else:
                    # if there are reachable corridors choose one of them randomly
                    next_room = random.choice(possible_corridors) 
                    print("THERE ARE NO REACHABLE URGENT ROOMS, give preference to CORRIDOS, chose -> ",next_room ,"among -> ", possible_corridors)
                return next_room
```


#### PlanToNext(self, check_battery): ####
This is a function to plan a path of waypoints to reach a random goal,by simply invoking the ```planner``` action server, and continuously checking that the battery does not get low during this phase. 

```bash
def PlanToNext(self, check_battery):
        environment_size = [3,3]
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, environment_size[0]),
                            y=random.uniform(0, environment_size[1]))
        #cancel previous existing request if any
        if(self.planner_client.is_running()):
            self.planner_client.cancel_goals()
        # Invoke the planner action server.
        self.planner_client.send_goal(goal)
        print("SIMULATE PLANNING A PATH TO REACH POSITION")
        # Wait for the action server computation and listen possible incoming stimulus about battery
        while not self.planner_client.is_done(): #Until planning has not finished
            # check the battery state
            if(self.is_battery_low() and check_battery == 1): # check_battery == 0 means robot has to go back to rech station
                print("### LOW BATTERY WHILE PLANNING THE PATH ###")
                return 0
            else: # if battery is not low OR robot is going back to rech stat, just plan a path
                print("---- Doing Path Planning ---")
                rospy.sleep(1)
        path_planned = self.planner_client.get_results().via_points
        return path_planned
```


#### SimulateMotionTime(self, plan, check_battery): ####
This function simulats travelling time to reach a goal through a given plan of waypoints, by invoking the ```controller``` action server and continuously checking that the battery does not get low during this phase. 
```bash
def SimulateMotionTime(self, plan, check_battery):
         # Start the action server for moving the robot through the planned via-points.
         goal = ControlGoal(via_points=plan)
         #cancel previous existing request if any
         if(self.controller_client.is_running()):
             self.controller_client.cancel_goals()
         self.controller_client.send_goal(goal)
         print('Following the planned path to reach the goal...')
         motion_completed = False
         while not rospy.is_shutdown():
             # If the battery is low, then cancel the control action server and take the `battery_low` transition.
             if(self.is_battery_low() and check_battery == 1): # check_battery == 0 means robot has to go back to rech station
                 print("### LOW BATTERY WHILE MOVING ALONG THE PLANNED PATH ###")
                 return motion_completed
             # If the controller finishes its computation,.
             if self.controller_client.is_done():
                 motion_completed = True
                 print(" MOTION COMPLETED ALONG THE PLANNED  PATH")
                 return motion_completed
             # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
             print(" ***** Moving along the planned path to reach goal... *****")
             rospy.sleep(1)
```

#### MoveToNext(self, new_position): ####
This is a simple function that manipulates the ontology to actually move the robot to a new location 
```bash
self.UpdateRobotTimestamp()
            self.LaunchReasoner()

            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()
            # MOVE ROBOT TO NEW LOCATION
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', new_position,current_room)
            print("ROBOT MOVED FROM ", current_room, " TO  ---> " , new_position)
```

#### Survey(self): ####
This function implements the algorithm of surveillance for which the robot stays in location room for 4 seconds if it is a ROOM, otherwise (if it is a corridor) it does not survey. In addition to that, here again the battery state is cchecked at each time to make sure it does not get low during this phase, if so, the surveillance is interrupted.
```bash
def Survey(self):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()
            survey_time = 0

            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()

            while(survey_time<=3):
                battery_state = self.is_battery_low()
                if(battery_state):
                    # if battery low occured
                    print("### LOW BATTERY WHILE DOING SURVEILLANCE ###")
                    print("stop survey and go to recharging...")
                    # return flag value for the FSM to indicate low_battery
                    return 0
                if(not self.IsCorridor(current_room)):
                    # if robot is in a room, survey for 4 sec
                    survey_time +=1
                    rospy.sleep(1)
                    print("Robot has been in the room for ",survey_time, "seconds", end ='\r')
                else:
                    # if robot is in a corridor, don't survey
                    print("Robot is in a corridor, will not survey...")
                    return 1
            print("I have surveilled the room for ",survey_time, "seconds")
            return 1
```

#### GoToRechargingStation(self): ####
This is a function to implement the algorithm to reach Recharging station taking into account the current position of the robot. Therefore here the robot checks if it can directly reach Location 'E', if so it moves towards it, otherwise he moves to corridors until he manages to see 'E' Location as ```reachable```. Note that here again the planning algorithm and Simulated Motion Time are again taken into account. 
```bash
def GoToRechargingStation(self):

        recharging_station = 'E'
        reached = 0
        check_battery = 0
        while (reached == 0):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()
            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()
            print("ROBOT IS IN ->",current_room," WITH LOW BATTERY")
            # ASK FOR REACHABLE ROOMS
            reachable_list = self.GetReachableRooms()
            # SEE IF ROBOT CAN REACH REACHARGING STATION
            if(recharging_station in reachable_list):
                print("++ Robot can reach Recharging Station ROOM from here ++")
                print("Plan a path...")
                path_planned = self.PlanToNext(check_battery)
                print("move along plan...")
                self.SimulateMotionTime(path_planned,check_battery)
                self.MoveToNext(recharging_station)
                reached = 1
                print("### Recharging Station reached, START RECHARGING ### ")
                return reached
            # OTHERWISE MOVE TO A CORRIDOR AND TRY AGAIN
            else:
                print("-- Robot can NOT reach Recharging Station ROOM from here --")
                print("First Move to a Corridor")
                print("Plan a path...")
                path_planned = self.PlanToNext(check_battery)

                print("move along plan...")
                self.SimulateMotionTime(path_planned,check_battery)

                for reachable_room in reachable_list:
                    if self.IsCorridor(reachable_room):
                        self.MoveToNext(reachable_room)
```



----------------------
### Robot-State node  : ### 
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements two ```services``` (i.e., ```state/set_pose``` and ```state/get_pose```) and a ```publisher``` (i.e., ```state/battery_low```) that runs on a separate thread. Here the modifications are mainly represented by some parameters changes related to the battery state, for which i set two different "timers" to set "charging/discharging" time, along with the elimination of some redundant part for my project. Therefore the structure of the node is pretty much inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation.



	
 

## Behaviuor Presentation
 
Here below you can find a short video (about 1 min), showing the actual workflow of this project, along with a brief text explanation of what is shown in order to fully understand what happens:

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/example.gif?raw=true" width="800px" align="center">
<p>
 
* First the **LoadOntology** State is executed loading the ontology and showing some prints.
* Then the FSM moves into **Decide** State, in which it prints useful info such as which are the currently reachable rooms or the urgent ones, the robot picks one of them (using the algorithms previously mentioned) and plans a path of a randome number of waypoints.
* **Surveillance** state is then executed: the robot simulates the motion time and after some seconds, actually moves to the selected location, since it is a corridor it will not survey but will pass immediately to **Decide** state again.
* previous steps is repeated until reaching a urgent ROOM (R3) in which the robot then stays for 4 second before going back to **Decide** state.
* the loop goes on until at a certain point (during "motion simulation" in the specific case of this video) robot gets low battery. At this point it interrupts the Surveillance state to move to the **Recharging** one.
* Robot therefore moves back until reaching recharging station (i.e 'E' location), where it waits for the battery to be fully charged by calling **Recharging** state multiple times
* When the battery gets finally high, the robot starts back its behaviour entering in the **Decide** state.
 ## Limitations and Possible Improvements
 
The main limitations of this project are related to the current implementation of ```Planning``` and ```Controlling``` algorithms. Here in fact, the code written in  [planner.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/planner.py) and in [controller.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/controller.py), only allow to ```simulate``` these actions, without actually thinking about how to reach a real room in the environment, but instead simulating the time waste of this planning action along with the one required for actually moving along the planned path. 
	
Therefore **possible improvements** could involve the implementation of a better algorithm that actually takes into account a real environment in order to plan some real waypoints to reach a real position, while regarding the controller node, a better algorithm could take information from a real robot to actually control it along the planned path.
	
In addition to this the implementation of the ```robot battery``` could be further improved, since for now it is just represented by a boolean value changing randomly its value after some predefined time, so a better algoritmh could be used to discharge the robot accordingly with the ```travelled distance``` for instance, which would also avoid the robot to reach locations that are too far from its recharging station, that would prevent him to go back and recharge its battery. Moreover another limition about it is that, if we use this particular robot simulaton within a wider map, it could happen that robot may actually receive the "recharged" battery state before actually reaching the Recharging station, therefore the algorithm based on "travelled distance" previously named could help also in this case. 

In the end, one last limitation is related to the "Urgency threshold" of the rooms, which causes them to become urgent after 7 second from their last visit: this collides with the Surveillance pattern implementation since, in the specific environment of this assignment, it causes the robot to remain "stucked" in a certain side of the map. In fact it will keep looping among the left side (R1-C1-R2) or the right one (R3-C2-R4), beacuse after having visited all the rooms of one side, the one that has been visited as first becomes urgent again requiring the robot to move there. therefore robot will change "side of the map" only after having returned to 'E ' location for recharging: here it randomly chooses one of the two corridors in which, by fact, it will remain until the next ```low battery``` occurrence. This could be simply avoided by increasing the threshold of the rooms or by changing the "planning" algorithm, in order to avoid visiting the same rooms multiple times in a row.
 

