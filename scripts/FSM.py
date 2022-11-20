#!/usr/bin/env python

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

global chosen_location
global path_planned

# define state Load Ontology
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

# define state Decide
class Decide(smach.State):
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        # rospy.sleep(2)
        print(' +++++ Executing state Decide +++++')
        global chosen_location
        global path_planned


        # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
        # self._helper.mutex.acquire() PROVO COMMENTARE ANCHW QUA
        try:
            chosen_location = self._helper.ChooseNextMove()
            if(chosen_location == 0): # se arriva 0 vuol dire che nell'helper e' arrivato low_battery
                return 'low_battery'
            else:
                print("NEXT ROOM WILL BE --> ", chosen_location) # altrimenti contiene stanza successiva
                # print("STATO BATTERIA false=carica True=scarica -->--", self._helper.is_battery_low())
                # rospy.sleep(2)
                path_planned = self._helper.PlanToNext(self.check_battery)
                if(path_planned == 0):
                    return 'low_battery'
                else:
                    # print('path planned successfully -> ', path_planned)
                    print('PATH PLANNED SUCCESSFULLY')
                    return 'decided'
        finally:
            # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
            # self._helper.mutex.release()
            rospy.sleep(0.1)
            # prof mette attesa qua fuori ma perch ha il planner, senza planner forse
            # manco il while ha senso..non aspetto che arrivi a destinazione ci arriva subito!
            # ma forse mi serve il while seeno da nessuna parte controllo piu volte il topic della batteria!!??

# define state Surveillance
class Surveillance(smach.State):
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        self.check_battery = 1
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        # time.sleep(2)
        print(' +++++ Executing state Surveillance +++++')
        global chosen_location #variabile globale per farmi passare la next room da stato decide,,cosi qui effettuo lo spostamento del robot
        # print("prova per veder se mi arriva var gobale..stanza e' ", chosen_location)
        global path_planned
        # self._helper.mutex.acquire() #################### PROVO A COMMENTARE IL MUTEX IN SURVEY PPER ORA
        # SIMULATE GOAL REACHING WITH CONTROLLER
        motion_completed = self._helper.SimulateMotionTime(path_planned,self.check_battery)
        if(motion_completed == 0): # se arriva 0 vuol dire che nell'helper e' arrivato low_battery
            # print("-------------------------------------SONO NELL IF LOW_BAT DELLA FSM")
            return 'low_battery'
        else: # altrimenti ho completato la simulazione, quindi sposto realmente il robot e vado avanti
            # ACTUALLY MOVE THE ROBOT TO THE CHOSEN LOCATION
            self._helper.MoveToNext(chosen_location)
            # print("STATO BATTERIA false=carica True=scarica -->--", self._helper.is_battery_low())
            # SURVEY LOCATION
            visited = self._helper.Survey()
            if(visited == 0): # se arriva 0 vuol dire che nell'helper e' arrivato low_battery
                # print("-------------------------------------SONO NELL IF LOW_BAT DELLA FSM")
                return 'low_battery'
            else:
                print("SURVEILLANCE OVER.. ")
                rospy.sleep(1)
                return 'visited'

# define state Recharging
class Recharging(smach.State):
    def __init__(self,my_helper):
        # initialisation function, it should not wait
        self._helper = my_helper
        smach.State.__init__(self,
                             outcomes=['loaded','decided','visited','low_battery','recharged'])
        self.time = 0
        self.station_reached = 0
        self.required_move = 1

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        print('+++++ Executing state Recharging +++++')

        # FIRST REACH THE RECHARGING STATION IN ORDER TO START THE RECHARGING PROCESS
        if(self.required_move):
            self.station_reached = self._helper.GoToRechargingStation()
            self.required_move = 0

        # print("STATION REACHED VALUe == ", self.station_reached)
        if(self.station_reached):
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if(self._helper.is_battery_low()): #finche la batteria e' scarica
                    print("I AM RECHARGING THE BATTERY-- ELSAPSED TIME: ", str(int(self.time)), "seconds", end ='\r')
                    self.time += 1
                    rospy.sleep(1) # metto un wait qua ok?
                    return 'low_battery'
                else:
                    print("BATTERY FULLY CHARGED AFTER : ", str(int(self.time)), "seconds") #print non funza
                    # reset variables
                    self.time = 0
                    self.station_reached = 0
                    self.required_move = 1
                    return 'recharged'
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(1)




def main():
    rospy.init_node('smach_example_MY_state_machine')

    my_helper = Helper()

    # Initialize robot position at the beginning of execution calling the init_pose function of the helper
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    my_helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    # sm.userdata.sm_counter = 0

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
    main()  # Initialise and start the ROS node.
