#!/usr/bin/env python

##
# \file FSM.py
# \brief Implement the Finite States Machine of robot's behaviour
# \author Claudio Del Gaizo
# \version 0.1
# \date 20/11/2022
#
#
# \details
#
# Subscribes to: <BR>
# [None]
#
# Publishes to: <BR>
# [None]
#
# Description:
#
# This node manages the overall behaviour of the robot by implementing a Finite States Machine:
# it is made of 4 different states (LoadOntology, Decide, Surveillance, Recharging), that are reached
# through 5 different transitions ('loaded','decided','visited','low_battery','recharged') and
# within  which it makes us of the functions defined in the Helper class contained in 'helper.py' script.
##

# import useful libraries
import roslib
import rospy
import smach
import smach_ros
import time
import random
from arch_skeleton import architecture_name_mapper as anm

# Import used messages defined within the ROS architecture.
from arch_skeleton.msg import Point, PlanAction, PlanGoal, ControlAction, ControlGoal

# define helper to use functions defined in 'helper.py'
from helper import Helper

 # ********************************PROVE PER SECONDO ASSIGNMENT********************************
from std_msgs.msg import UInt32
from assignment2.msg import RoomConnection
from assignment2.srv import RoomInformation
 # ********************************PROVE PER SECONDO ASSIGNMENT********************************


# __Global variables__ to pass data between states
#
# global chosen_location : the 'next location' decided in 'DECIDE' state
#
# global path_planned : The path planned in 'DECIDE' state to reach next location

global chosen_location
global path_planned

# ********************************PROVE PER SECONDO ASSIGNMENT********************************
global received_id
global prec
received_id = 0
prec = 0
def IdCallback(id):

    """
    Callback that receives the marker ID detected bt 'marker_publish' and shares it with 'marker_derver',
    managing the response of the latter to actually build the map/ontology

    """
    #global img
    #img = img_msg
    print('SONO NELLA CALLBACK')
    rospy.loginfo("visto id= @[%i] dal subscriber", id.data);
    global received_id
    received_id = id.data
    rospy.loginfo("received_id= @[%i] ", received_id);

    #per chiamare il client passando l id in teoria basta questo
    #response_info = id_client(id.data)
    #print('received_id stampa risposta ', response_info) #FUNZIONA OK

# ********************************PROVE PER SECONDO ASSIGNMENT********************************


# define state Load Ontology
    ##
    #
    #
    # Init State 'LoadOntology' in which the robot loads the map of the enviroment
    # ...
    # Attributes
    # ----------
    # __helper__ : Helper
    # an object of class Helper contained in helper.py to use its functions
    #
    # Methods
    # -------
    # __init__(self) :
    #
    # initialise class
    #
    # __execute__(self, userdata):
    #
    # called on execution, Loads the ontolgy calling a function of the helper and
    # returns 'loaded' transition to move to 'DECIDE' state
    ##
class LoadOntology(smach.State):

    ##
    # called on initialisation
    # ...
    # Parameters
    # ----------
    # __helper__: Helper
    #
    # an object of class Helper contained in helper.py to use its functions
    ##
    def __init__(self, my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper

        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])


    ##
    # called on execution, Loads the ontolgy calling '_helper.MY_LoadOntology()' and
    # returns 'loaded' transition to move to 'DECIDE' state
    # ...
    # Parameters
    # ----------
    # __helper__ : Helper
    #
    # an object of class Helper contained in helper.py to use its functions
    ##
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        print('+++++ Executing state LoadOntology +++++')
        ################################self._helper.MY_LoadOntology()  DECOMMENTaRE
        ################################print('Loading Ontology...')

        ################################rospy.sleep(4)
        ################################return 'loaded
        # ********************************PROVE PER SECONDO ASSIGNMENT********************************

        print('trial di subscribtion...')
        global received_id
        global prec
        rospy.loginfo("received_id= @[%i] ", received_id);
        doors_list = []
        connections_list = []

        if(received_id != 0 and received_id != prec):
            print("ho ricevuto un id marker,,--> %i ",received_id) #METTO SERVICE CALL QUA? ma poi come gestisco ontology=?
            prec = received_id
            #per chiamare il client passando l id in teoria basta questo POTREI METTERE TUTTO NELLA CALLBACK IN REALTA E IN QUESTO STATO METTERE SOLO IF==14 return loaded cosi lo stato effettivamente aspetta che sia caricata la ontology! per ora faccio cosi per received_idre debug poi posso modificare
            response_info = id_client(received_id)
            name = response_info.room
            x = response_info.x
            print('valore x centro = ',x)
            y = response_info.y
            print('valore x centro = ',y)

            for item in response_info.connections:
                doors_list.append(item.through_door)

            for item in response_info.connections:
                connections_list.append(item.connected_to)
            if(received_id == 14):
                last_call = 1
            else:
                last_call = 0

            self._helper.MY_LoadOntology(name,x,y,doors_list,connections_list,last_call)

        if(received_id == 14):
            print("ultimo marker detectato posso passare a fase succ ",received_id)
            rospy.sleep(4)
            return 'loaded'
        rospy.sleep(2)
        return 'recharged'
        # ********************************PROVE PER SECONDO ASSIGNMENT********************************




##
# State 'Decide' in which the robot decides which will be the next move
# ...
# Attributes
# ----------
# __helper__ : Helper
#
# an object of class Helper contained in helper.py to use its functions
#
# __check_battery__ : boolean
#
# flag to state wether to check the battery state or not (1=yes 0=NO)
#
# Methods
# -------
# __init__(self):
#
# initialise class
#
# __execute__(self, userdata):
#
# called on execution, chooses next move calling helpers'function and plans a path
# to reach such goal, then returns 'low_battery' transition if battery gets low
# during this phase, otherwise returns 'decided' transition to move to Surveillance state
##
# define state Decide
class Decide(smach.State):

    ##
    # called on initialisation
    # ...
    # Parameters
    # ----------
    # __helper__ : Helper
    #
    # an object of class Helper contained in helper.py to use its functions
    #
    # __check_battery__ : boolean
    #
    # flag to state wether to check the battery state or not (1=yes 0=NO)
    ##
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
    ##
    # called on execution, chooses next move calling '_helper.ChooseNextMove()'' and plans a path
    # to reach such goal calling '_helper.PlanToNext()', then returns 'low_battery' transition if
    # battery gets low during this phase, otherwise returns 'decided' transition to move to
    # Surveillance state
    # ...
    # Parameters
    # ----------
    # __chosen_location__ : str
    #
    # location decided by 'Decide' state
    #
    # __path_planned__ : PlanGoal
    #
    # path plan computed by 'Decide' state
    ##
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
                path_planned = self._helper.PlanToNext(self.check_battery,chosen_location)
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

##
# State 'Surveillance' in which the robot moves to a location and surveys it
# ...
# Attributes
# ----------
# __helper__ : Helper
#
# an object of class Helper contained in helper.py to use its functions
#
# __check_battery__ : boolean
#
# flag to state wether to check the battery state or not (1=yes 0=NO)
#
# Methods
# -------
# __init__(self):
#
# initialise class
#
# __execute__(self, userdata):
#
# called on execution, moves to next room calling helpers'function simulating time waste
# to reach such goal, then returns 'low_battery' transition if battery gets low
# during this phase, otherwise returns 'visited' transition to move to 'Decide' state
##
# define state Surveillance
class Surveillance(smach.State):
        ##
        # called on initialisation
        # ...
        # Parameters
        # ----------
        # __helper__ : Helper
        #
        # an object of class Helper contained in helper.py to use its functions
        #
        # __check_battery__ : boolean
        #
        # flag to state wether to check the battery state or not (1=yes 0=NO)
        ##
    def __init__(self,my_helper):


        # initialisation function, it should not wait
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
        ##
        # called on execution, moves to next room calling helpers'functions simulating time waste
        # to reach such goal. After that it applies survey algorithm calling 'Survey' function.
        # then it returns 'low_battery' transition if battery gets low
        # during these phases, otherwise returns 'visited' transition to move to 'Decide' state
        # ...
        # Parameters
        # ----------
        # __chosen_location__ : str
        #
        # location decided by 'Decide' state
        #
        # __path_planned__ : PlanGoal
        #
        # path plan computed by 'Decide' state
        ##
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
                return 'visited'
##
# State 'Recharging' in which the robot moves to Recharging Station and waits for the battery to be fully charged
# ...
# Attributes
# ----------
# __helper__ : Helper
#
# an object of class Helper contained in helper.py to use its functions
#
# __time__ : int
#
# counter to know time required for recharging
#
# __station_reached__ : boolean
#
# flag to state if robot reached Recharging station or not  (1=yes 0=NO)
#
# Methods
# -------
# __init__(self):
#
# initialise class
#
# __execute__(self, userdata):
#
# called on execution, moves to Recharging Station calling helpers' function,
# then waits for the battery to be fully charged. The wait is implemented through
# repetitive calls of the 'low_battery' transitions.
# When battery is charged, returns 'recharged' transition to move to 'Decide' State
##
# define state Recharging
class Recharging(smach.State):

    ##
    # called on initialisation
    # ...
    # Parameters
    # ----------
    # __helper__ : Helper
    #
    # an object of class Helper contained in helper.py to use its functions
    #
    # __time__ : int
    #
    # counter to know time required for recharging
    #
    # __station_reached__ : boolean
    #
    # flag to state if robot reached Recharging station or not (1=yes 0=NO)
    ##
    def __init__(self,my_helper):


        # initialisation function, it should not wait
        self._helper = my_helper
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
        self.time = 0
        self.station_reached = 0
    ##
    # called on execution, moves to Recharging Station calling helpers' function,
    # then waits for the battery to be fully charged. The wait is implemented through
    # repetitive calls of the 'low_battery' transitions.
    # When battery is charged, returns 'recharged' transition to move to 'Decide' State
    # ...
    # Parameters
    # ----------
    # __helper__ : Helper
    #
    # an object of class Helper contained in helper.py to use its functions
    #
    # __time__ : int
    #
    # counter to know time required for recharging
    #
    # __station_reached__ : boolean
    #
    # flag to state if robot reached Recharging station or not (1=yes 0=NO)
    ##
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



##
# Main program entry, initialises the node, the robot position calling a function of the helper,
# and the Finite States Machine using SMACH libraries. In the end launches rospy.spin()
##
def main():

    rospy.init_node('smach_example_MY_state_machine')

    my_helper = Helper()

    # Initialize robot position at the beginning of execution calling the init_pose function of the helper
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    my_helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # ********************************PROVE PER SECONDO ASSIGNMENT********************************
    #initialize subscriber to /my_ID_topic
    rospy.Subscriber("/my_ID_topic", UInt32,  IdCallback)


    # ********************************PROVE PER SECONDO ASSIGNMENT********************************

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LOAD ONTOLOGY', LoadOntology(my_helper),
                               transitions={'loaded':'DECIDE',
                                            'decided':'LOAD ONTOLOGY',
                                            'visited':'LOAD ONTOLOGY',
                                            'low_battery':'LOAD ONTOLOGY',
                                            'recharged':'LOAD ONTOLOGY'}) #qui ho cancellato acnhe il remapping delle variabili ch enon servono credo
        smach.StateMachine.add('DECIDE', Decide(my_helper),
                               transitions={'loaded':'DECIDE',
                                            'decided':'SURVEILLANCE',
                                            'visited':'DECIDE',
                                            'low_battery':'RECHARGING',
                                            'recharged':'DECIDE'})
        smach.StateMachine.add('SURVEILLANCE', Surveillance(my_helper),
                               transitions={'loaded':'SURVEILLANCE',
                                            'decided':'SURVEILLANCE',
                                            'visited':'DECIDE',
                                            'low_battery':'RECHARGING',
                                            'recharged':'SURVEILLANCE'})
        smach.StateMachine.add('RECHARGING', Recharging(my_helper),
                               transitions={'loaded':'RECHARGING',
                                            'decided':'RECHARGING',
                                            'visited':'RECHARGING',
                                            'low_battery':'RECHARGING',
                                            'recharged':'DECIDE'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
 # ********************************PROVE PER SECONDO ASSIGNMENT********************************
 #initialize client for /room_info service (marker_server)
   rospy.wait_for_service("/room_info")
   id_client= rospy.ServiceProxy("/room_info", RoomInformation)


 # ********************************PROVE PER SECONDO ASSIGNMENT********************************
   main()  # Initialise and start the ROS node.
