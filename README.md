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

#### MODIFIED FOR ASSIGNMENT 2 ####
**Added Features**: now this node also subscribes to ```/my_ID_topic``` to receive the marker id detected by robot's camera and put it into a global variable; it also implements the client for ```/room_info``` to be able to send request with the given ID and receive as response room informations.

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/FSM.png?raw=true" width="400" />
<p>

#### LoadOntology State: ####
#### MODIFIED FOR ASSIGNMENT 2 ####
	
This is the ```Init state``` of the FSM, within which it waits for the robot to pass through the detection of all markers. Once a new marker is detected it calls  ```/room_info``` service giving the detected ID as argument, then it puts the related room information received as response into some variables and passes them to the helper's function ```"MY_loadOntolgy"```, which will add the room to the ontology map. It repeats this step for each room (cycling the state by calling  ```recharging ``` transition) until last room has been added to the ontology; then it returns the ```'loaded'``` transition in order to move to the successive state i.e. ```Decide```.

```bash
class LoadOntology(smach.State):

    def __init__(self, my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper

        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])

    def execute(self, userdata):
        print('+++++ Executing state LoadOntology +++++')

        global received_id
        global prec
        rospy.loginfo("received_id= @[%i] ", received_id);
        doors_list = []
        connections_list = []

        if(received_id != 0 and received_id != prec):
            print("ho ricevuto un id marker,,--> %i ",received_id)
            prec = received_id
            # call /room_info service of 'marker_server' to receive info about the room associated with id given as input
            response_info = id_client(received_id)
            # put service response into variables
            name = response_info.room
            x = response_info.x
            y = response_info.y

            for item in response_info.connections:
                doors_list.append(item.through_door)

            for item in response_info.connections:
                connections_list.append(item.connected_to)

            if(received_id == 14):
                last_call = 1
            else:
                last_call = 0
            # call MY_LoadOntology to create the map
            self._helper.MY_LoadOntology(name,x,y,doors_list,connections_list,last_call)

        if(received_id == 14):
            print("*** MAP COMPLETED *** passing to the next state ",received_id)
            rospy.sleep(4)
            return 'loaded'
        rospy.sleep(2)
        return 'recharged'	
```
_Please note that the other classes of this node has not been further modified with respect to previous implementation, exception made for some change of variables name that do not have effect on the algorithms but are just more correlated with the new project._
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

In this state the robot moves to reach the goal through the waypoints planned in ```Decide state```, calling helper's function  ```MoveRobot```, then it updates its position in the ontology calling helpers'function ```UpdateRobotPosition``` which manipulates the ontology. After that it applies the surveillance algorithm of the new room by calling helper's function ```Survey```. In the end it returns ```'low_battery'``` transition if battery gets low during these phases, (to move to ```Recharging``` state), otherwise returns ```'visited'``` transition to move back to ```'Decide'``` state.

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
        motion_completed = self._helper.MoveRobot(path_planned,self.check_battery)

        if(motion_completed == 0):
            # low battery occured
            return 'low_battery'
        else:
            # plan followed successfully
            self._helper.UpdateRobotPosition(chosen_location)
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
#### MODIFIED FOR ASSIGNMENT 2 ####
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, adapting to the needs of my project, and implements an action server named ```motion/controller```. In particular i've added the client for ```/move_base``` Action Service to actually move a real robot to a specified target (received by the planner), and two functions to call such Action sending the target (```_reach_pose_client()```) or an empty goal (```MY_CancGoal()```) to stop the robot. In addition to that I added the ```DoneCallback()``` definition for ```move_base``` to allow canceling correctly the goal of move_base in case  the ```controller``` receives a cancel request.
Here you can find the core of the code, but for further details about the previoulsy named functions please give a look at [controller.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/controller.py).
	
 ```bash

class ControllingAction(object):

    def __init__(self):
        self.isActive = 0
        # Initalise action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      arch_skeleton.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
    def execute_callback(self, goal):
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        target = 0 #boolean to only send the target instead of both starting point and target
        for point in goal.via_points:
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                self._as.set_preempted()
                return

            if(target): #only send the target goal
                self._reach_pose_client(point) 
                while (self.isActive and not rospy.is_shutdown()):
                     if self._as.is_preempt_requested():
                         rospy.loginfo(anm.tag_log('*********  Service has been cancelled by the client! *********', LOG_TAG))
                         # Actually cancel this service.
                         self._as.set_preempted()
                         self.move_base_client.cancel_all_goals() 
                         self.isActive = 0 
                         return

                     rospy.loginfo(anm.tag_log('wating until goal is reached OR CANCELLED...', LOG_TAG))
                     rospy.sleep(1)
            else:
                target += 1
                rospy.loginfo(anm.tag_log('AVOID SENDING CURRENT POSITION AS NEW GOAL', LOG_TAG))


            feedback.reached_point = point
            self._as.publish_feedback(feedback)


        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        print('result', result)
        self._as.set_succeeded(result)
        return  # Succeeded.

 ```
	

----------------------	
### Planner node  : ###
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements an action server named ```motion/planner```. Here the modifications are mainly represented by some parameters changes, therefore the structure of the node is inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation

#### MODIFIED FOR ASSIGNMENT 2 ####
This node has only been slighlty modified with respect to the first version: now it does not waste time anymore to simulate path planning because it is not a "dummy" implementation, and instead of a plan composed by a random set of points, it returns a path plan containing the starting position of the robot and the goal to reach.


----------------------
### Helper classes  : ###
#### MODIFIED FOR ASSIGNMENT 2 ####
As previously said, the script ```helper.py``` contains the implementation of two different helper classes: ```ActionClientHelper``` & ```Helper```. The first one is a class to simplify the implementation of a client for ROS action servers, and has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository in order to implement the planning and controlling action, so please check the linked repository for further documentation.


The second one instead is an helper class I made to simplify the interaction with the ontology through aRMOR, and therefore to define useful functions to be used in the Finite states machine script (```FSM.py```). Here you can find the definition of the subscriber to ```/state/battery_low``` topic to receive continuous information about the battery state previously named in ```FSM node```, as well as the definition of the```clients``` to ```/motion/planner``` and ```/motion/controller``` Services.


**Here follows the code explanation of the main functions of this class that have been modified/added** with respect to the first version of this project, please find a more detailed description of non-modified functions in the ```Readme.md``` of ```main``` branch or directly in [helper.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/helper.py):

----------------------
#### MY_LoadOntology(self,room_name, x_value, y_value, doors_linked,connections,last_call)  : ####
#### MODIFIED FOR ASSIGNMENT 2 ####
This function has been heavily modified with respect to former implementation.
Now it creates the map adding all the locations one by one according to the arguments given as inputs, such as room name, center coordinates and so on. It adds the required properties to the ontology ```/topological_map.owl``` by using the "manipulation" tool of aRMOR_api client, then saves the result in a new file named ```MY_topological_map.owl```. 
Please note that with this implementation it is no more necessary to loop the process three times (as it happened in my first implementation), probably due to the different properties added to the ontology, that allow aRMOR_api to work better, so the previous while loop is now canceled. 
	
```bash
def MY_LoadOntology(self,room_name, x_value, y_value, doors_linked,connections,last_call): #repeat_doors = 0/1

            # ADD ALL OUR AXIOMS
            if(client.manipulation.add_ind_to_class(room_name, "LOCATION")): #mettere if per checcare
                print("Added" + room_name + "to LOCATION")

            # ADD HASDOOR PROPERTIES and DOOR INDIVUDUALS
            # Distinction between rooms and corridors
            for item in doors_linked:
                client.manipulation.add_objectprop_to_ind("hasDoor", room_name, item)
                print("Added" +item+ 'to hasDoor of'  + room_name)
                client.manipulation.add_ind_to_class(item, "DOOR")

            # ADD CONNECTIONS PROPERTIES
            for item in connections:
                client.manipulation.add_objectprop_to_ind("connectedTo", room_name, item)
                print("Added" +item+ 'to Connections of' + room_name )

            #ADD center coordinates
            client.manipulation.add_dataprop_to_ind("X_center",room_name,'Float',str(float(x_value)))
            client.manipulation.add_dataprop_to_ind("Y_center",room_name,'Float',str(float(y_value)))

            # ADD TIMESTAMPS TO ROOMS
            if(not "C" in room_name and not "E" in room_name): #only add timestamps to rooms not corridors
            	client.manipulation.add_dataprop_to_ind("visitedAt",room_name,'Long',str(int(time.time())))
                # done_once = 1
            if(last_call): #to add this only once
                client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
                print("Robot set in E location")
                # DISJOINT INDIVIDUALS
                client.manipulation.disj_inds_of_class("LOCATION")
                client.manipulation.disj_inds_of_class("DOOR")


            # APPLY CHANGES
            client.utils.apply_buffered_changes()
            # client.utils.sync_buffered_reasoner()

            # SAVE AND EXIT
            client.utils.save_ref_with_inferences("/root/ros_ws/src/topological_map/MY_topological_map.owl")
```



#### PlanToNext(self, check_battery): ####
#### MODIFIED FOR ASSIGNMENT 2 ####
This is a function to plan a path of waypoints to reach a **given** **goal**,by simply invoking the ```planner``` action server, and continuously checking that the battery does not get low during this phase. 
In this version it does not consider envirnoment size but creates a plan of two waypoints: a starting point (i.e the current robot position) and the target point (i.e the center coordinates of the ```target_location``` given as input)

```bash
 def PlanToNext(self, check_battery, target_location):
        room_coordinates = self.GetRoomCoordinates(target_location)
        goal = PlanGoal()
        goal.target = room_coordinates

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


#### MoveRobot(self, plan, check_battery): ####
#### MODIFIED FOR ASSIGNMENT 2 ####
This function takes the place of former ```SimulateMotionTime``` function, which simulated travelling time to reach a goal through a given plan of waypoints, by invoking the ```controller``` action server and continuously checking that the battery does not get low during this phase. Now it does not simulate anymore but actually moves the robot by invoking the new version of the ```controller``` already described. Conceptually the remaining functioning is the same as before. 
```bash
def MoveRobot(self, plan, check_battery):
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
                 print("### LOW BATTERY WHILE MOVING ALONG THE PLANNED PATH, Cancel goal... ###")
                 # send a cancel_all_goals to 'controller' server of controller.py
                 self.controller_client.cancel_goals()
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

#### UpdateRobotPosition(self, new_position): ####
This function is exactly the old ```MoveToNext()```, just renamed for a more immediate understanding of its functioning: it is a simple function that manipulates the ontology to update robot's current location.
```bash
def UpdateRobotPosition(self, new_position):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()

            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()
            # MOVE ROBOT TO NEW LOCATION
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', new_position,current_room)
            print("ROBOT MOVED FROM ", current_room, " TO  ---> " , new_position)
```

#### Survey(self): ####
#### MODIFIED FOR ASSIGNMENT 2 ####
The first version of this function implemented an algorithm of surveillance for which the robot stayed in location room for 4 seconds if it is a ROOM, otherwise (if it is a corridor) it did not survey. In addition to that, here again the battery state is checked at each time to make sure it does not get low during this phase, if so, the surveillance is interrupted.

**The new version of this function** removes the 4 seconds waiting and replaces it with a camera rotation of 360 degrees, done with a client to ```/MY_move_arm``` which calls the service by giving pose number 10, which causes the camera to have the desired rotation.
```bash
def Survey(self):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()
            survey_time = 0


            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()

            battery_state = self.is_battery_low()
            if(battery_state):
                # if battery low occured
                print("### LOW BATTERY WHILE DOING SURVEILLANCE ###")
                print("stop survey and go to recharging...")
                # return flag value for the FSM to indicate low_battery
                return 0
            if(not self.IsCorridor(current_room)):

                # ****** if robot is in a room, survey rotating the camera ******
                print("Survey room by rotating the camera of 360 degrees")
                # wait for My_move arm server
                rospy.wait_for_service('/MY_move_arm')
                # initailise client
                camera_client = rospy.ServiceProxy('/MY_move_arm', MY_SetPose)
                # rotate by calling predefined pose = 10
                camera_client(10)
                #wait fot camera rotation
                rospy.sleep(6)

            else:
                # if robot is in a corridor, don't survey
                print("Robot is in a corridor, will not survey...")
                return 1
            return 1
```

#### GoToRechargingStation(self): ####
#### MODIFIED FOR ASSIGNMENT 2 ####
This is a function to implement the algorithm to reach Recharging station taking into account the current position of the robot. Therefore here the robot checks if it can directly reach Location 'E', if so it moves towards it, otherwise he moves to corridors until he manages to see 'E' Location as ```reachable```. Note that here again the planning algorithm and Simulated Motion Time are again taken into account. 

In the new version i've added the case in which the robot gets low battery while already being in Location 'E' (which can occur with the real simulation of the second assignment) and simply changed the names of the functions called previousòy explained (i.e SimulateMotionTime -> MoveRobot and MoveToNext -> UpdateRobotPosition).
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
            if(current_room == recharging_station): ###### MODIFICATO AGGIUNGENDO QUESTO CASO
                reached = 1
                print("### ALREADY in Recharging Station, START RECHARGING ### ")
                return reached

            # ASK FOR REACHABLE ROOMS
            reachable_list = self.GetReachableRooms()
            # SEE IF ROBOT CAN REACH REACHARGING STATION
            if(recharging_station in reachable_list):
                print("++ Robot can reach Recharging Station ROOM from here ++")
                print("Plan a path...")
                path_planned = self.PlanToNext(check_battery,recharging_station)
                print("move along plan...")
                self.MoveRobot(path_planned,check_battery)
                self.UpdateRobotPosition(recharging_station)
                reached = 1
                print("### Recharging Station reached, START RECHARGING ### ")
                return reached
            # OTHERWISE MOVE TO A CORRIDOR AND TRY AGAIN
            else:
                print("-- Robot can NOT reach Recharging Station ROOM from here --")
                print("First Move to a Corridor")

                for reachable_room in reachable_list:
                    if self.IsCorridor(reachable_room):
                        print("Plan a path...")
                        path_planned = self.PlanToNext(check_battery,reachable_room)
                        print("move along plan...")
                        self.MoveRobot(path_planned,check_battery)

                        self.UpdateRobotPosition(reachable_room)
```



----------------------
### Robot-State node  : ### 
#### MODIFIED FOR ASSIGNMENT 2 ####
This node has been taken from [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, slightly adapting to the needs of my project, and implements one ```service``` (i.e., ```state/set_pose```) and a ```publisher``` (i.e., ```state/battery_low```) that runs on a separate thread. Here the modifications are mainly represented by some parameters changes related to the battery state, for which i set two different "timers" to set "charging/discharging" time, along with the elimination of some redundant part for my project. Therefore the structure of the node is pretty much inviariated, for this reason please consult [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository for further documentation.
**Modified Feature**: in the new version, this node does not present the server for ```state/get_pose```, but instead implements a subscriber to ```/odom``` in order to update the current robot position with the real one taken by the odometry. The remaining functions are invariated, please look at [robobt_state.py](https://github.com/claudio-dg/assignment_1/blob/second_assignment_changes/scripts/robot_state.py) for further details.



	
 

## Behaviuor Presentation
 
For the Demo of the firs assignment please give a look to  the Readme.md of [assignment_1](https://github.com/claudio-dg/assignment_1/blob/main/README.md) (main branch), while for the final Demo of the whole project see Readme.md of [assignment2](https://github.com/claudio-dg/assignment2/blob/main/README.md).
 

