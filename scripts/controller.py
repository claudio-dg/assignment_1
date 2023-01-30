#! /usr/bin/env python

##
# \file controller.py
# \brief Implement the Action server for controlling action
# \author arch_skeleton
# \version 0.1
# \date 20/11/2022
#
#
# \details
#
# Client for Action Service:
#
# /move_base: to let the robot reach the given target
#
# Server for Action:
#
# /motion/controller : Action to control robot along the the path
#
# Description:
#
# Credits of this node to the arch_skeleton repository <https://github.com/buoncubi/arch_skeleton.git>
# which has been modified in order to adapt to this case. In particular i've added the client for /move_base Action Service
# to actually move a real robot, two functions to call such Action sending the target or an empty goal to stop the robot, and as
# DoneCallback definition for move_base to allow canceling correctly the goal of move_base in case 'controller' receives a cancel request
#
##

import random
import rospy
import actionlib
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from arch_skeleton.msg import ControlFeedback, ControlResult
from arch_skeleton.srv import SetPose
import arch_skeleton  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# for move_base
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

LOG_TAG = anm.NODE_CONTROLLER # A tag for identifying logs producer.


## An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
##
class ControllingAction(object):

    def __init__(self):
        self.isActive = 0
        # Initalise action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      arch_skeleton.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan containing vurrent position and target),
    # and it calls move_base action to let the robot rech the target (i.e the last point of the list)
    # As soon as the target is reached, it returns the succeded state, but if robots gets low battery
    # while reaching the goal, such target is canceled and communication aborted.
    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        target = 0 #boolean to only send the target instead of both starting point and target
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return

            if(target): #only send the target goal

                # function call to sent goal to move_base
                self._reach_pose_client(point) #chiamata la funzione in cui c'e il client di move_base che invia il goal all action

                # aspetta in un loop finche il goal non e raggiunto o cancellato (i.e. finche la comunicazione con action move_base e' attiva)
                # wait in a loop until goal is reached or canceled (i.e until communication with move_base is active)
                while (self.isActive and not rospy.is_shutdown()):
                     if self._as.is_preempt_requested(): # SE ARRIVA UNA RICHIESTA DI CANCEL al controller (OSSIA QUA CI ARRIVO QUANDO IN HELPER.py viene chiamato self.controller_client.cancel_goals() riga 691 !!!!!)
                         rospy.loginfo(anm.tag_log('*********  Service has been cancelled by the client! *********', LOG_TAG))
                         # Actually cancel this service.
                         self._as.set_preempted() #cancello controller service (ma questo non fa fermare il robot, serve anche quello dopo)

                         self.move_base_client.cancel_all_goals() # NOTA, questo mi fa entrare nella DONE_CALLABACK in cui chiamo funzione che manda goal vuoto a move_base per cancellare DAVVERO il goal precedente!!!!!
                         self.isActive = 0 #in realta ridondante perche lo setto anche nella calback ma per sicurezza..
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

# rosmsg show actionlib_msgs/GoalStatusArray -->actionlib_msgs/GoalStatus[] status_list
  # uint8 PENDING=0
  # uint8 ACTIVE=1
  # uint8 PREEMPTED=2
  # uint8 SUCCEEDED=3
  # uint8 ABORTED=4
  # uint8 REJECTED=5
  # uint8 PREEMPTING=6
  # uint8 RECALLING=7
  # uint8 RECALLED=8
  # uint8 LOST=9


    # This method implements the action client for move_base and sends the target in order to move the real robot
    def _reach_pose_client(self,pose):


        # Eventually, wait for the server to be initialised.
        rospy.loginfo(anm.tag_log('waiting for server /move_base..', LOG_TAG))
        self.move_base_client.wait_for_server()

        self.isActive = 1 # communication is now active (serve per fare entrare nel while riga 96)

        rospy.loginfo(anm.tag_log('building goal to send to move_base', LOG_TAG))
        target = MoveBaseGoal()
        # build the MoveBaseGoal msg
        target.target_pose.header.frame_id = "map"
        target.target_pose.header.stamp = rospy.Time.now()
        target.target_pose.pose.orientation.w = 1
        target.target_pose.pose.position.x = pose.x
        target.target_pose.pose.position.y = pose.y
        rospy.loginfo(anm.tag_log('sending goal to move_base', LOG_TAG))
        # Sends the goal to the move_base action server.
        self.move_base_client.send_goal(target,self.DoneCallback)
        # NOTA BENE: in questo modo (ossia mettendo self.DoneCallback), faccio in modo che quando azione finisce venga chiamata la funzione done_callback
        # da me scritta, all interno della quale cancello realmente il goal mandando un nuvo goal vuoto (perche solo con cancel_all_goals non funzionava,
        # continuava a raggiungere il goal precedente) 


    # function to send an EMPTY goal to move_base in order to cancel the previous one and actually stop the robot
    def MY_CancGoal(self):

        rospy.loginfo(anm.tag_log(' ########## buiding EMPTY TARGET to send to move_base ##########', LOG_TAG))
        target = MoveBaseGoal()
        # build the empty MoveBaseGoal msg to cancel previous goal and stop the robot
        target.target_pose.header.frame_id = ""
        rospy.loginfo(anm.tag_log('sending EMPTY target to move_base', LOG_TAG))
        # Sends the goal to the move_base action server.
        self.move_base_client.send_goal(target,self.DoneCallback)

    # Function executed when the communication ends. It calls MY_CancGoal if a cancel is requested (due to low battery)
    # to actually cancel the goal and stop the robot, and sets self.isActive to zero to state that communication is ended.
    def DoneCallback(self, status, result):
        # Prints on the info window the status returned by the action server communication.
        if status == 2: #PREEMPTED
            rospy.loginfo("received a cancel request in DONE_CALLABACK.")
            # this stops the robot
            self.MY_CancGoal()
            self.isActive = 0   # The action client communication is not active.
            return

        if status == 3:
            rospy.loginfo("GOAL REACHED")
            self.isActive = 0   # The action client communication is not active.
            return

        if status == 4:
            rospy.loginfo("Goal was aborted.")
            self.isActive = 0   # The action client communication is not active.
            return

        if status == 5:
            rospy.loginfo("Goal has been rejected.")
            self.isActive = 0   # The action client communication is not active.
            return

        if status == 8:
            rospy.loginfo("Goal received a cancel request.")
            self.isActive = 0   # The action client communication is not active.
            return


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
