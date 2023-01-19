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
# Client for Services:
#
# /state/set_pose : Service to set robot position ####### QUA DEVO METTERE CLIENT A move_base Action perche anizche settare un pose e basta..tramite move_base faccio effettivamente muovere il robot
#
# Server for Action:
#
# /motion/controller : Action to control robot along the the path ####### motion va lasciato qua
#aggiungere publisher a joint1 per  far ruotare poi la telecamera di 36
#
# Description:
#
# Credits of this node to the arch_skeleton repository <https://github.com/buoncubi/arch_skeleton.git>
# which has only slightly been modified in order to adapt to this case
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
        # Get random-based parameters used by this server
        #### self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0]) QUESTO NON SERVE PIU

        # Initalise action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) #/move_base

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      arch_skeleton.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        # print(f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay '
        #            f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                ##### NON SERVE PIU Wait before to reach the following via point. This is just for testing purposes.
            # delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            # rospy.sleep(delay)


            # Set the new current position into the `robot-state` node.
            self._reach_pose_client(point) ##### al posto di que devo mettere spostamento vero e proprio
                                    # ma chiamo uguale cosi lascio invariato!!
            # Publish a feedback to the client to simulate that a via point has been reached.

            feedback.reached_point = point
            self._as.publish_feedback(feedback)


            # Log current robot position.
            log_msg = f'####### Reaching point ({point.x}, {point.y}).########'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
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


# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
    def _reach_pose_client(self,pose):
        # Eventually, wait for the server to be initialised.
        # rospy.wait_for_service(anm.SERVER_SET_POSE)
        # rospy.loginfo(anm.tag_log('STO ASPETTANDO SERVER /move_base..', LOG_TAG))

        # self.move_base_client.wait_for_server() ######ATTESA CANCELLATA PROVO
        rospy.loginfo(anm.tag_log('costruisco goal da mandare a mov_base', LOG_TAG))
        target = MoveBaseGoal()
        # build the MoveBaseGoal msg
        target.target_pose.header.frame_id = "map"
        target.target_pose.header.stamp = rospy.Time.now()
        target.target_pose.pose.orientation.w = 1
        target.target_pose.pose.position.x = pose.x
        target.target_pose.pose.position.y = pose.y
        rospy.loginfo(anm.tag_log('Invio goal a mov_base', LOG_TAG))
        # Sends the goal to the move_base action server.
        self.move_base_client.send_goal(target) #  ,self.DoneCallback)

# def _set_pose_client(pose):
#     try:
#         # Log service call.
#         log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
#         rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
#         # Call the service and set the current robot position.
#         service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
#         service(pose)  # The `response` is not used.
#     except rospy.ServiceException as e:
#         log_msg = f'Server cannot set current robot position: {e}'
#         rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
