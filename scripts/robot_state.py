#!/usr/bin/env python

##
# \file robot_state.py
# \brief Implement the RobotState class
# \author Claudio Del Gaizo
# \version 0.1
# \date 20/11/2022
#
#
# \details
#
#
# Publishes to: <BR>
# 
# /state/battery_low : to publish battery state
#
# Subscribes to: <BR>
# 
# /odom : to get Robot position
# 
# Server for Service:
#
# /state/get_pose : to allow retrieving robot position
# /state/set_pose : to allow modifying robot position -- No more in the updated version
#
# Description:
#
# This node contains the implementation of RobotState class, which defines two Service servers for
# to get and set the current robot pose, and a publisher to notify that the battery is low and
# which runs on a separate thread.
#
#
##

# import useful libraries
import threading
import random
import rospy

# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from arch_skeleton.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
from arch_skeleton.msg import Point

#to retrieve real pose of robot_state
from nav_msgs.msg import Odometry

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE

##
# The node manager class. This class defines two services to get and set the current
# robot pose, a publisher to notify that the battery is low, and a subscriber to get odometry values of the robot.
# ...
# Attributes
# ----------
# __pose__ : Point
# contains the coordinates (x,y) of robot position
#
# __battery_low __: boolean
# flag containing battery state (True=Low False=high)
#
# __randomness__ : boolean
# flag to enable randomness
#
# Methods
# -------
# __init__(self):
#
# initalises the node and the main attributes, defines the service servers and starts publishing on a separate thread
#
#
# __set_pose_callback__(self, request).
#
# The `robot/set_pose` service callback implementation.
# The `request` input parameter is the current robot pose to be set,
# as given by the client. This server returns an empty `response`.
#
# UpdateRobotPose_Callback(self, odom):
#
# The `/odom` topic callback implementation, simply update self.pose with current robot position taken from Odometry
#
# __get_pose_callback__(self, request):
#
# The `robot/get_pose` service callback implementation.
# The `request` input parameter is given by the client as empty. Thus, it is not used.
# The `response` returned to the client contains the current robot pose.
#
# __is_battery_low__(self):
#
# Publish changes of battery levels. This method runs on a separate thread.
#
#  __random_battery_notifier__(self, publisher):
#
# Publish when the battery changes state (i.e., high/low) based on a random
# delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]` for discharging,
# `self._random_battery_time[2]`, `self._random_battery_time[3] for recharging).
# The message is published through the `publisher` input parameter and is a
# boolean value, i.e., `True`: battery low, `False`: battery high.
##
class RobotState:

    ##
    # initalises the node and the main attributes, defines the service servers and starts publishing on a separate thread
    # ...
    # Parameters
    # ----------
    # __pose__ : Point
    #
    # robot position
    #
    # __time__ : int
    #
    # counter to know time required for recharging
    #
    # __battery_low__ : boolean
    #
    # flag to state if robot battery is low or not (`True`: battery low, `False`: battery high.)
    #
    #  __randomness__: boolean
    #
    #  flag to state if enabling randomness or not
    ##
    def __init__(self):
        # Initialise this node.
        rospy.init_node('robot_state_manager')
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [550.0, 750.0, 400.0,450.0])# [550.0, 750.0, 325.0,450.0][25.0, 45.0, 20.0,35.0] The delay between changes of battery levels, i.e., high/low.
                                                                                             # It should be a list `[min_time, max_time]`, and the battery level change
                                                                                             # will occur after a random number of seconds within such an interval.
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose_callback)

        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose_callback)

        # Define subscriber to /odom to get Robot positions
        rospy.Subscriber("/odom", Odometry, self.UpdateRobotPose_Callback)


        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()


    ##
    # The `robot/set_pose` service callback implementation.
    # ...
    # ----------
    # \param request : Point
    #
    # current robot pose to be set
    #
    # \return SetPoseResponse: SetPoseResponse()
    #
    # empty response
    #
    ##
    def set_pose_callback(self, request):
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    ##
    # The `/odom` topic callback implementation, simply update self.pose with current robot position taken from Odometry
    # ...
    # ----------
    # \param odom : odometry
    #
    # current robot odmetry
    #
    # \return [None]
    #
    ##
    def UpdateRobotPose_Callback(self, odom):
        self._pose = Point(odom.pose.pose.position.x,odom.pose.pose.position.y)


    ##
    # The `robot/get_pose` service callback implementation.
    # ...
    # ----------
    # \param request : Point
    #
    # input parameter is given by the client as empty. thus it's not used
    #
    # \return response: GetPoseResponse()
    #
    # contains the current robot pose.
    #
    ##
    # def get_pose_callback(self, request):
    #     # Log information.
    #     if self._pose is None:
    #         rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
    #     # Create the response with the robot pose and return it.
    #     response = GetPoseResponse()
    #     response.pose = self._pose
    #     return response

    def get_pose_callback(self, request):
        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    ##
    # Publish changes of battery levels on anm.TOPIC_BATTERY_LOW. This method runs on a separate thread.
    # ...
    # ----------
    #  \param [none]
    #
    # \return [none]
    #
    #
    ##
    def _is_battery_low(self):
        # Define a `latched` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self._random_battery_notifier(publisher)

    ##
    # Publish when the battery changes state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]` for discharging,
    #`self._random_battery_time[2]`, `self._random_battery_time[3] for recharging).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    # ...
    # ----------
    #  \param publisher : Publisher()
    #
    # \return [none]
    #
    #
    ##
    def _random_battery_notifier(self, publisher):
        discharge_delay = 0  # Initialised to 0 just for logging purposes.
        recharging_delay = 0
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                recharging_delay = random.uniform(self._random_battery_time[2], self._random_battery_time[3])
                # Wait for simulate battery recharging
                rospy.sleep(recharging_delay)
            else:
                discharge_delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
                # Wait for simulate battery usage.
                rospy.sleep(discharge_delay)
            # Change battery state.
            self._battery_low = not self._battery_low

            publisher.publish(Bool(self._battery_low))



if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
