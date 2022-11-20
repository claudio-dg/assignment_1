#! /usr/bin/env python3
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
# Initializing with buffered manipulation and reasoning

###client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
client.utils.load_ref_from_file("/root/ros_ws/src/topological_map/topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
class ActionClientHelper:
    # Class constructor, i.e., class initializer. Input parameters are:
    #  - `service_name`: it is the name of the server that will be invoked by this client.
    #  - `action_type`: it is the message type that the server will exchange.
    #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
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

    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
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

    # Stop the computation of the action server.
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

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
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

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
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

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None





class Helper:

    def __init__(self):
        print("init helper")
        # Create a shared mutex to synchronize the usage of battery
        self.mutex = Lock()
        # Set the initial state of the `self._battery_low` variable
        self._battery_low = False
        # Define the callback associated with the battery low ROS subscribes.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        # qui creo un oggetto della classe ACTIONCLIENTHELPER dentro a classe helper!!!
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    # def reset_battery():  qurato secoono me a me non serve oerche io chiamo is_battery_low gua qui nell helper quindi qui variabile e sempre aggiornata!!!! credo!
    #     self._battery_low = False
    def MY_LoadOntology(self):
        i = 1 # variabile per far ripetere tutto lo script 3 volte --> CAUSA BUG CHE NON SO COME RISOLVERE E COSI, LANCIANDOLO TRE VOLTE DVREBBE FUNZO-->SCRIVEE IN REAMDE!!
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
                # ADD timestamps L IF NON FUNZIONA
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

   # function to launch the reasoner and reason about the ontology
    def LaunchReasoner(self):
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    # function to update robot timestamp each time the robot moves or a query is required
    def UpdateRobotTimestamp(self):
        timestamp = client.query.dataprop_b2_ind('now','Robot1')
        old_time = timestamp[0]
        initial = old_time.find('"') +1
        final = old_time.find('"', initial)
        old_time = old_time[initial : final]
        # print(old_time)
        client.manipulation.replace_dataprop_b2_ind("now", "Robot1", 'Long', str(int(time.time())), old_time)
        # self.LaunchReasoner()

    def UpdateCurrentRoomTimestamp(self, current_room):

        if(not self.IsCorridor(current_room)): # Update the timestamp only if it is a room (not a corridor)
            timestamp = client.query.dataprop_b2_ind('visitedAt',current_room)
            old_time = timestamp[0]
            initial = old_time.find('"') +1
            final = old_time.find('"', initial)
            old_time = old_time[initial : final]

            client.manipulation.replace_dataprop_b2_ind('visitedAt', current_room,'Long', str(int(time.time())) ,old_time)

    def IsCorridor(self,input_room):
        # ASK FOR CORRIDORS
        results_Corridors = client.query.ind_b2_class("CORRIDOR")
        corridors_list = []
        for item in results_Corridors:
            initial = item.find('#') +1
            final = item.find('>', initial)
            name = item[initial : final]
            corridors_list.append(name)
        # print('CORRIDORS -->', corridors_list)

        # CHECK if input_room belongs to the corridor list
        if(input_room in corridors_list):
            # print(input_room, "e' un corridoio")
            return True
        else:
            return False

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
                print("### LOW BATTERY WHILE DECIDING NEXT LOCATION ###")
                return 0

            elif not urgents_list: #se NON ci sono stanze urgenti--> vado in una a caso controllando prediligendo pero prima i corridoi
                for reachable_room in reachable_list:
                    if self.IsCorridor(reachable_room):
                        possible_corridors.append(reachable_room)
                    else:
                        possible_rooms.append(reachable_room)
                if(not possible_corridors): # se non ci sono corridoi raggiungibili
                    next_room = random.choice(possible_rooms) #vado in una room raggiungibile a caso
                    print("THERE ARE NO URGENT ROOMS NOR CORRIDOR -> chose ",next_room ,"among -> ", possible_rooms)
                else: #se invece ci sono corridoi
                    next_room = random.choice(possible_corridors) #vado in un corridoio raggiungibile a casp
                    print("THERE ARE NO URGENT ROOMS -> chose",next_room ," among -> ", possible_corridors)
                return next_room

            else: # mentre se invece ci sono stanze urgenti ---controllo se una di queste e reachable e se si ci vado, altrimetni vado in una reachable a caso prediligendo corridoi
                for urgent_room in urgents_list:
                    if(urgent_room in reachable_list): #se una urgente e' reachable-->ci vado
                        possible_rooms.append(urgent_room)
                if(possible_rooms):  # quindi se c'e almeno una urgent reachable, prendi una di queste a caso e ci vado
                    next_room = random.choice(possible_rooms)
                    print("THERE ARE REACHABLE URGENT ROOMS-> chose ",next_room ,"among -> ", possible_rooms)
                    return next_room

                for reachable_room in reachable_list: #se sono qui  vuol dire che urgenti non sono reachable quindi scelgo corridoi se ci sono altrimenti room a caso (COPIATO CODICE FATTO SOPRA)
                    if self.IsCorridor(reachable_room):
                        possible_corridors.append(reachable_room)
                    else:
                        possible_rooms.append(reachable_room)
                if(not possible_corridors): # se non ci sono corridoi raggiungibili
                    next_room = random.choice(possible_rooms) #vado in una room raggiungibile a caso
                    print("THERE ARE NO REACHABLE URGENT ROOMS NOR CORRIDORS,  chose -> ",next_room ,"among -> ", possible_rooms)
                else: #se invece ci sono corridoi
                    next_room = random.choice(possible_corridors) #vado in un corridoio raggiungibile a casp
                    print("THERE ARE NO REACHABLE URGENT ROOMS, give preference to CORRIDOS, chose -> ",next_room ,"among -> ", possible_corridors)
                return next_room # mentre se non ci sono corridoi, prendo l ultima della lista e vado in quella (QUI SI PUO MIGLIORARE IMPLEMENTANDO UNA VARIABILE RANDOMICA CHE VA DA 0 A len(reachablelist) e prendo quella)

    #  funzione chiamata dopo il Choose NExt che simula la creazione di path_plan perdendo tempo di fatto
    def PlanToNext(self, check_battery):
        # Get the environment size from ROS parameters.
        # environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        environment_size = [3,3] #proco cosi perceh con param fa casino
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, environment_size[0]),
                            y=random.uniform(0, environment_size[1]))
        # Invoke the planner action server.
        if(self.planner_client.is_running()):
            self.planner_client.cancel_goals() #cancel previous existing request if any
        self.planner_client.send_goal(goal)
        print("SIMULATE PLANNING A PATH TO REACH POSITION")
        # Wait for the action server computation and listen possible incoming stimulus about battery
        while not self.planner_client.is_done(): #Until planning has not finished
            # check the battery state
            if(self.is_battery_low() and check_battery == 1): # check_battery == 0 means robot has to go back to rech station
                print("### LOW BATTERY WHILE PLANNING THE PATH ###")
                return 0
            else: # if batery is not low OR robot is going back to rech stat, just plan a path
                print("---- Doing Path Planning ---")
                # print("finito = ",self.planner_client.is_done())
                rospy.sleep(1)
        # print("FINISHED PAth Planning")
        path_planned = self.planner_client.get_results().via_points
        return path_planned


    def SimulateMotionTime(self, plan, check_battery):
         # Start the action server for moving the robot through the planned via-points.
         goal = ControlGoal(via_points=plan)
         if(self.controller_client.is_running()):
             self.controller_client.cancel_goals() #cancel previous existing request if any
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



    def MoveToNext(self, new_position):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()

            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()
            # MOVE ROBOT TO NEW LOCATION
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', new_position,current_room)
            # self.UpdateRobotTimestamp()
            # self.LaunchReasoner()
            print("ROBOT MOVED FROM ", current_room, " TO  ---> " , new_position)



    def Survey(self):
            self.UpdateRobotTimestamp()
            self.LaunchReasoner()
            survey_time = 0
            # ASK FOR CURRENT ROBOT POSITION
            current_room = self.GetCurrentRoom()

            while(survey_time<=3): # 5---3
                battery_state = self.is_battery_low() # false=carica True = scarica--provo a metterlo qua per vedere se cosi si aggiorna sto valore e entra nell if
                # print("STATO DELLA BATTERIA NEL CICLO WHILE ===> " ,battery_state)
                if(battery_state): #per prima cosa checko ogni volta che non si scarichi, se si scarica esci e ritorna un valore
                    print("### LOW BATTERY WHILE DOING SURVEILLANCE ###")
                    print("stop survey and go to recharging...")
                    return 0 #flag per indicare low battery....in fsm faccio if result = 0__return 'low_battery'...if =1 ad esempio return 'visited'
                if(not self.IsCorridor(current_room)): #se e' una room la sorveglio (RPOVO IF ANZI CHE ELIF..)
                    survey_time +=1
                    rospy.sleep(1)
                    print("Robot has been in the room for ",survey_time, "seconds", end ='\r')
                else: # sono in un corridoio,
                    print("Robot is in a corridor, will not survey...")
                    # print("SONO In un CORRIDOIO, non sorveglio e passo a stato Decide")
                    return 1
            print("I have surveilled the room for ",survey_time, "seconds")
            #se sono qui vuol dire che sono passati 5 secondi e non si e scaricato il robot, quindi aggionro atimestap e tutto
            # self.UpdateCurrentRoomTimestamp(current_room)
            # self.LaunchReasoner()
            return 1


        # The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
    def _battery_callback(self, msg):
            print("################# BATTERY STATE HAS CHANGED ###############")
            # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
            self.mutex.acquire()
            try:
                # Get the battery level and set the relative state variable encoded in this class.
                self._battery_low = msg.data
                # print("STATO LOW_BATTERY IN CALLBACK??? -->--", msg.data)
                # Uncomment below to log data.
                if self._battery_low:
                    print("robot has low battery")
                    # log_msg = 'Robot with low battery.'
                else:
                    print("robot battery fully charged")
                    # log_msg = 'Robot battery fully charged.'
                # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            finally:
                # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
                self.mutex.release()

        # Get the state variable encoded in this class that concerns the battery level.
        # The returning value will be `True` if the battery is low, `False` otherwise.
        # Note that the node using this class might exploit the `reset_state` function to improve robustness.
        # Also note that this function should be used when the `mutex` has been acquired. This assures the
        # synchronization  with the threads involving the subscribers and action clients.
    def is_battery_low(self):
            return self._battery_low

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
                # AGGIUNGERE ANCHE SIMULAZIONE TEMPO PERCORRENZA
                # QUESTO RENDERA NECESSARIO AUMENTARE IL TEMPO DI RICARICA!!
                # PER EVITARE CHE ROBOT SI CARICHI PRIMA DI ARRIVARE IN E!!!
                print("move along plan...")
                self.SimulateMotionTime(path_planned,check_battery)
                self.MoveToNext(recharging_station)
                reached = 1
                print("### Recharging Station reached, START RECHARGING ### ")
                return reached
            # OTHERWISE MOVE TO A CORRIDOR AND TRY AGAIN
            else:
                print("-- Robot can NOT reach Recharging Station ROOM from here --")
                print("First Move to a Corridor") #questo pezzo e' specifico della mia ontology, so che andando in u  corridoio poi posso andare in # -*- coding: utf-8 -*-
                                                  #E so anche che corridoi sono sempre raggiungibili quando E non lo e'
                                                  # ossia quando robot e' in una ROOM
                print("Plan a path...")
                path_planned = self.PlanToNext(check_battery)
                # AGGIUNGERE ANCHE SIMULAZIONE TEMPO PERCORRENZA
                # QUESTO RENDERA NECESSARIO AUMENTARE IL TEMPO DI RICARICA!!
                # PER EVITARE CHE ROBOT SI CARICHI PRIMA DI ARRIVARE IN E!!!

                print("move along plan...")
                self.SimulateMotionTime(path_planned,check_battery)

                for reachable_room in reachable_list: #se sono qui  vuol dire che urgenti non sono reachable quindi scelgo corridoi se ci sono altrimenti room a caso (COPIATO CODICE FATTO SOPRA)
                    if self.IsCorridor(reachable_room):
                        self.MoveToNext(reachable_room)



    def GetCurrentRoom(self):
        current_position = client.query.objectprop_b2_ind("isIn", 'Robot1')
        # for item in results_rooms:
        current_room = current_position[0]
        initial = current_room.find('#') +1
        final = current_room.find('>', initial)
        current_room = current_room[initial : final]
        return current_room

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


    # Update the current robot pose stored in the `robot-state` node.
    @staticmethod
    def init_robot_pose(point):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)  # None that the service `response` is not used.
            log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        except rospy.ServiceException as e:
            err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
            rospy.logerr(anm.tag_log(err_msg, LOG_TAG))
