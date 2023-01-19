#! /usr/bin/env python3
##
# \file helper.py
# \brief Implement helper classes for actions and for interfacing Armor
# \author Claudio Del Gaizo
# \version 0.1
# \date 20/11/2022
#
#
# \details
#
# Subscribes to: <BR>
#
#  /state/battery_low : to retrieve battery state
#
# Publishes to: <BR>
# [None]
#
# Service CLient to : <BR>
#
# armor_interface_srv : to interact with ontology through ARMOR api
#
# Action CLient to : <BR>
#
# /motion/planner : Action to plan the path
#
# /motion/controller : Action to control robot along the the path
#
# Description:
#
# This script is the core of the project and contains all the main function used by the actions and the Finite State machine
# in order to reach the desired behaviour of the robot. Therefore the functions of these classes are called in the other scripts
# for instance to allow loading the ontology, as well as applying manipulations or querys to it.
##


import rospy
from actionlib import SimpleActionClient
# Import the armor client class
from armor_client import ArmorClient
import time
import random

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import ROS-based messages.
from std_msgs.msg import Bool
from arch_skeleton.msg import PlanAction, ControlAction
from arch_skeleton.srv import SetPose
from arch_skeleton.msg import Point, PlanGoal, ControlGoal
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
LOG_TAG = 'HELPER'
# Global Definition of Armor CLient
client = ArmorClient("test2","ontology2")

# load starting ontology
client.utils.load_ref_from_file("/root/ros_ws/src/topological_map/topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

##
# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
# CREDITS for this class to <https://github.com/buoncubi/arch_skeleton/wiki/Noetic-Py3-Solution> whose code has been
# reused here to simplify the implementation of a client.
#
##
class ActionClientHelper:
    ##
    # Class constructor, i.e., class initializer. Input parameters are:
    #
    #  \param service_name`: it is the name of the server that will be invoked by this client.
    #
    #  \param action_type: it is the message type that the server will exchange.
    #
    #  \param done_callback: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #
    #  \param  feedback_callback: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #
    #  \param  mutex: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
    ##
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    ## Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    ## Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    ## Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    ## This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    ##This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()

    ## Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    ## Get `True` if the action server is running, or `False` otherwise.
    # note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    ## Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None




##
# A class to decouple the implementation of the Finite State Machine to the stimulus that might
# lead to state transitions. This class manages the synchronization with subscribers and action
# servers, and defines all the functions to interact with the ontology through the ARMOR api, which
# are then called directly from the Finite States Machine in 'FSM.py'
# ...
#
# Methods
# -------
# _init_ (self):
#
# Class constructor. Initialise subscriber and action clients
#
# MY_LoadOntology (self)
#
# Creates the map adding all the locations and required properties, then saves the ontology in the specified path
#
# LaunchReasoner (self)
#
# apllies changes and launches reasoner of the ontology
#
# UpdateRobotTimestamp (self)
#
# update robot timestamp each time the robot moves or a query is required
#
# UpdateCurrentRoomTimestamp (self, current_room)
#
# update 'VisitedAt' property of a room
#
# IsCorridor (self, input_room)
#
# states if location given as input is a corridor or not
#
# ChooseNextMove (self)
#
# Implement algorithm to decide the next ove of the robot
#
# PlanToNext (self, check_battery)
#
#  Plan a path to reach next goal
#
# SimulateMotionTime (self, plan, check_battery)
#
# simulate Travelling time to reach a goal through a given plan
#
# MoveToNext (self, new_position)
#
# Move the robot to another room
#
# Survey (self)
#
# implement the algoritmh of room surveillance
#
# is_battery_low (self)
#
# gives the current state of the battery
#
# GoToRechargingStation (self)
#
# implement algorithm to reach recharging station from current postion
#
# GetCurrentRoom (self)
#
# gives the current position of the robot
#
# GetReachableRooms (self)
#
# get the list of currently reachable rooms
#
# GetUrgentRooms (self)
#
# get the list of currently urgent rooms
##
class Helper:
    ##
    # Class constructor. Initialise subscriber and action clients
    # ...
    # ----------
    # \param [None]
    #
    # \return [None]
    #
    ##
    def __init__(self):
        print("init helper")
        # Create a shared mutex to synchronize the usage of battery
        self.mutex = Lock()
        # Set the initial state of the `self._battery_low` variable
        self._battery_low = False
        # Define the callback associated with the battery low ROS subscribes.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)
    ##
    # Creates the map adding all the locations and required properties, then saves the ontology in the specified path
    # ...
    # ----------
    # \param [None]
    #
    # \return [None]
    #
    ##
    """
    def AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACANCELLARETUTTOPRIMA_MY_LoadOntology(self):
        i = 1 # Counter to repeat the function 3 times in order to make the loading ontology actually work (please see README.md for further details)
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

            i +=1  #increase counter"""
    def MY_LoadOntology(self,room_name, x_value, y_value, doors_linked,connections,last_call): #repeat_doors = 0/1
        # i = 3 #1 # Counter to repeat the function 3 times in order to make the loading ontology actually work (please see README.md for further details)
        # done_once = 0 # to load timestamps only once
        # while i <=3:
            # ADD ALL OUR AXIOMS
            if(client.manipulation.add_ind_to_class(room_name, "LOCATION")): #mettere if per checcare
                print("Added" + room_name + "to LOCATION")


            # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
            #client.manipulation.disj_inds_of_class("LOCATION")

            # ADD PROPERTIES TO OBJECTS
            # Distinction between rooms and corridors
            for item in doors_linked:
                client.manipulation.add_objectprop_to_ind("hasDoor", room_name, item)
                print("Added" +item+ 'to hasDoor of'  + room_name)
                client.manipulation.add_ind_to_class(item, "DOOR")# FUNZIONA!!

            for item in connections:
                client.manipulation.add_objectprop_to_ind("connectedTo", room_name, item)
                print("Added" +item+ 'to Connections of' + room_name )

            #ADD center coordinates
            client.manipulation.add_dataprop_to_ind("X_center",room_name,'Float',str(float(x_value)))
            client.manipulation.add_dataprop_to_ind("Y_center",room_name,'Float',str(float(y_value)))

                # INITIALIZE ROBOT POSITION QUESTA COME LA DEVO METTERE?????????????
            # client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")

                # ADD timestamps in questo modo pero le aggiungo anche ai corridoi..boh provo vediamo
            # if(done_once == 0):
            if(not "C" in room_name and not "E" in room_name): #only add timestamps to rooms not corridors
            	client.manipulation.add_dataprop_to_ind("visitedAt",room_name,'Long',str(int(time.time())))
                # done_once = 1
            if(last_call): #to add this only once
                client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E") # LO metto in E?? dove lo metto!?!?!?
                print("Robot set in E location")
                client.manipulation.disj_inds_of_class("LOCATION")
                client.manipulation.disj_inds_of_class("DOOR") # FUNZIONA!!


            # APPLY CHANGES
            client.utils.apply_buffered_changes()
            # client.utils.sync_buffered_reasoner()

            # SAVE AND EXIT
            client.utils.save_ref_with_inferences("/root/ros_ws/src/topological_map/MY_topological_map.owl")

            # i +=1  #increase counter
    ##
    # function to launch the reasoner and reason about the ontology
    # ...
    # ----------
    # \param [None]
    #
    # \return [None]
    #
    ##
    def LaunchReasoner(self):
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()


    ##
    # function to update robot timestamp each time the robot moves or a query is required
    # ...
    # ----------
    # \param [None]
    #
    # \return [None]
    #
    ##
    def UpdateRobotTimestamp(self):
        timestamp = client.query.dataprop_b2_ind('now','Robot1')
        old_time = timestamp[0]
        initial = old_time.find('"') +1
        final = old_time.find('"', initial)
        old_time = old_time[initial : final]
        client.manipulation.replace_dataprop_b2_ind("now", "Robot1", 'Long', str(int(time.time())), old_time)



    ##
    # function to update property 'VisitedAt' of input Room
    # ...
    # ----------
    # \param current_room
    #
    # room to which update the timestamp
    #
    # \return [None]
    #
    ##
    def UpdateCurrentRoomTimestamp(self, current_room):

        if(not self.IsCorridor(current_room)): # Update the timestamp only if it is a room (not a corridor)
            timestamp = client.query.dataprop_b2_ind('visitedAt',current_room)
            old_time = timestamp[0]
            initial = old_time.find('"') +1
            final = old_time.find('"', initial)
            old_time = old_time[initial : final]

            client.manipulation.replace_dataprop_b2_ind('visitedAt', current_room,'Long', str(int(time.time())) ,old_time)

    ##
    # function to determine if given location is a corridorr or not
    # ...
    # ----------
    # \param input_room
    #
    # the location for which asking to the ontology if being a corridor or not
    #
    # \returns True=IsaCorridor False=IsNOTaCorridor
    #
    ##
    def IsCorridor(self,input_room):
        # ASK FOR CORRIDORS
        results_Corridors = client.query.ind_b2_class("CORRIDOR")
        corridors_list = []
        for item in results_Corridors:
            initial = item.find('#') +1
            final = item.find('>', initial)
            name = item[initial : final]
            corridors_list.append(name)

        # CHECK if input_room belongs to the corridor list
        if(input_room in corridors_list):
            return True
        else:
            return False

    ##
    # implement the algorithm with which the robot decides what location to visit next
    # ...
    # ----------
    # \param [None]
    #
    # \returns next_room :
    #
    # the location to go next
    #
    ##
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
                    if(urgent_room in reachable_list): #se una urgente e' reachable-->ci vado
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
                    next_room = random.choice(possible_rooms) #vado in una room raggiungibile a caso
                    print("THERE ARE NO REACHABLE URGENT ROOMS NOR CORRIDORS,  chose -> ",next_room ,"among -> ", possible_rooms)
                else:
                    # if there are reachable corridors choose one of them randomly
                    next_room = random.choice(possible_corridors) #vado in un corridoio raggiungibile a casp
                    print("THERE ARE NO REACHABLE URGENT ROOMS, give preference to CORRIDOS, chose -> ",next_room ,"among -> ", possible_corridors)
                return next_room



    ##
    # function to extract info about center coordinates of a room
    # ...
    # ----------
    # \param location_name : string -
    #
    # contains the name of the room
    #
    # \returns room_coordinates : Point():
    #
    # the coordinates of the center of the input room
    #
    ##
    def GetRoomCoordinates(self, location_name):
        x= 0
        y= 0
        x_coord = client.query.dataprop_b2_ind('X_center',location_name)
        for item in x_coord:
            initial = item.find('"') +1
            final = item.find('"^', initial)
            x = float(item[initial : final])

        y_coord = client.query.dataprop_b2_ind('Y_center',location_name)
        for item in y_coord:
            initial = item.find('"') +1
            final = item.find('"^', initial)
            y = float(item[initial : final])
        room_coordinates = Point(x,y)

        return room_coordinates
    ##
    # function to Plan a path of waypoints to reach next goal
    # ...
    # ----------
    # \param check_battery : boolean -
    #
    # determines if battery must be checked or not while planning
    #
    # \returns path_planned : PlanGoal():
    #
    # the determined waypoints of the plan to reach the goal
    #
    ##
    def PlanToNext(self, check_battery, target_location):
        # environment_size = [3,3]
        room_coordinates = self.GetRoomCoordinates(target_location)
        goal = PlanGoal()
        goal.target = room_coordinates
        # goal.target = Point(x=random.uniform(0, environment_size[0]), #########qui devo mettere il target da mandare al planner, ossia le coordinate della chosen location!!!!
        #                     y=random.uniform(0, environment_size[1]))
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

    ##
    # function to simulate Travelling time to reach a goal through a given plan of waypoints
    # ...
    # ----------
    # \param plan : PlanGoal():
    #
    # The plan of waypoints to follow obtained from 'PlanToNext()' function
    #
    # \param check_battery : boolean -
    #
    # determines if battery must be checked or not while simulating
    #
    # \returns [None]
    #
    ##
    def SimulateMotionTime(self, plan, check_battery): # PRATICAMENTE DEVO SOLO CAMBIARE IL NOME DI STA QUA RENDERLO: MOVE TO NEXT E la moveToNExt la chiama Update
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


    ##
    # function to move robot to a new position by querying the ontology
    # ...
    # ----------
    # \param new_position : string:
    #
    # the name of the new location where to move the robot
    #
    # \returns [None]
    #
    ##
    def MoveToNext(self, new_position):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()

            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()
            # MOVE ROBOT TO NEW LOCATION
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', new_position,current_room)
            print("ROBOT MOVED FROM ", current_room, " TO  ---> " , new_position)


    ##
    # function to implement the algorithm of location's Surveillance
    # ...
    # ----------
    # \param [None]
    #
    # \returns [None]
    #
    ##
    def Survey(self):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()
            survey_time = 0
            ##################################################################################################################################
            #################### POSSO METTERE QUA LA ROTAZIONE DELLA TELECAMERA SENZA FARE IL CASINO DELLA DoneCallbak???####################
            ##################################################################################################################################
            # METTO QUA ROTAZIONE AL POSTO DELL'ATTESA WHILE E ME NE FREGO SE SI SCARICA MENTRE E QUA????????

            
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


    ##
    # Callback function to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
    # ...
    # ----------
    # \param msg: boolean
    #
    #  contains boolean related to battery state: True=low False=High
    #
    # \returns [None]
    #
    ##
    def _battery_callback(self, msg):
            print("################# BATTERY STATE HAS CHANGED ###############")
            # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
            self.mutex.acquire()
            try:
                # Get the battery level and set the relative state variable encoded in this class.
                self._battery_low = msg.data
                if self._battery_low:
                    print("robot has low battery")
                else:
                    print("robot battery fully charged")
            finally:
                # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
                self.mutex.release()

    ##
    #  function to get the state variable encoded in this class that concerns the battery level.
    # ...
    # ----------
    # \param [None]
    #
    #  contains boolean related to battery state: True=low False=High
    #
    # \returns _battery_low: boolean
    #
    #  contains boolean related to battery state: True=low False=High
    ##
    def is_battery_low(self):
            return self._battery_low


    ##
    # function to implement the algorithm to reach Recharging station taking into account from current position
    # ...
    # ----------
    # \param [None]
    #
    # \returns [None]
    #
    ##
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
                path_planned = self.PlanToNext(check_battery,recharging_station)
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

                for reachable_room in reachable_list:
                    if self.IsCorridor(reachable_room):
                        print("Plan a path...")
                        path_planned = self.PlanToNext(check_battery,reachable_room)
                        print("move along plan...")
                        self.SimulateMotionTime(path_planned,check_battery)

                        self.MoveToNext(reachable_room)


    ##
    # function to get current room by querying the ontology with ARMOR Api
    # ...
    # ----------
    # \param [None]
    #
    # \returns current_room: string
    #
    # name of the Location in which the robot is currently in
    #
    ##
    def GetCurrentRoom(self):
        current_position = client.query.objectprop_b2_ind("isIn", 'Robot1')
        current_room = current_position[0]
        initial = current_room.find('#') +1
        final = current_room.find('>', initial)
        current_room = current_room[initial : final]
        return current_room

    ##
    # function to get the list of currently reachable rooms by querying the ontology with ARMOR Api
    # ...
    # ----------
    # \param [None]
    #
    # \returns reachable_list: string[]
    #
    # list of currently reachable rooms
    #
    ##
    def GetReachableRooms(self):
        reachable_list = []
        # ASK FOR REACHABLE ROOMS
        results = client.query.objectprop_b2_ind("canReach", "Robot1")
        # PUT REACHABLE ROOMS IN A LIST AND PRINT THEM
        for item in results:
            initial = item.find('#') +1
            final = item.find('>', initial)
            name = item[initial : final]
            reachable_list.append(name)
        return reachable_list

    ##
    # function to get the list of currently URGENT rooms by querying the ontology with ARMOR Api
    # ...
    # ----------
    # \param [None]
    #
    # \returns urgents_list: string[]
    #
    # list of currently URGENT rooms
    #
    ##
    def GetUrgentRooms(self):
        urgents_list = []
        # ASK FOR REACHABLE ROOMS
        results_Urgents = client.query.ind_b2_class("URGENT")
        # PUT URGENT ROOMS IN A LIST
        for item in results_Urgents:
            initial = item.find('#') +1
            final = item.find('>', initial)
            name = item[initial : final]
            urgents_list.append(name)
        return urgents_list


    ##
    # function to Update the current robot pose stored in the `robot-state` node.
    # ...
    # ----------
    # \param [None]
    #
    # \returns [None]
    #
    #
    ##
    @staticmethod
    def init_robot_pose(point):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)  # Note that the service `response` is not used.
            log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        except rospy.ServiceException as e:
            err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
            rospy.logerr(anm.tag_log(err_msg, LOG_TAG))
